import numpy as np
import math
from pxr import Gf, Sdf, Usd, UsdGeom

def is_interactive_mode():
    try:
        hou.ui
        return True
    except AttributeError:
        return False

def ray_box_intersection(ray, bbox):
    bbox_range = bbox.GetRange()
    intersects, enter_dist, exit_dist = ray.Intersect(bbox_range)
    if intersects:
        return True, enter_dist
    return False, float('inf')

def create_occlusion_buffer(stage, camera_prim, resolution, selected_prims, far_clip_mult=1.0, max_occl_culled_prims=None):
    # Get the camera
    camera = UsdGeom.Camera(camera_prim)
    
    # Get camera parameters
    camera_transform = camera.GetLocalTransformation()
    camera_position = camera_transform.ExtractTranslation()
    
    # Get camera frustum parameters
    horizontal_aperture = camera.GetHorizontalApertureAttr().Get()
    vertical_aperture = camera.GetVerticalApertureAttr().Get()
    focal_length = camera.GetFocalLengthAttr().Get()
    near_clip, far_clip = camera.GetClippingRangeAttr().Get()
    far_clip *= far_clip_mult

    # Calculate field of view
    fov_horizontal = 2 * np.arctan((horizontal_aperture * 0.5) / focal_length)
    fov_vertical = 2 * np.arctan((vertical_aperture * 0.5) / focal_length)

    # Create a bounding box cache
    bbox_cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])

    # Create an occlusion buffer
    occlusion_buffer = np.full(resolution, far_clip, dtype=np.float32)

    # Initialize counter for processed pixels
    processed_pixels_count = 0

    # Iterate through all pixels
    for y in range(resolution[1]):
        for x in range(resolution[0]):
            # Calculate ray direction
            ndc_x = (x / resolution[0] - 0.5) * 2
            ndc_y = (y / resolution[1] - 0.5) * 2

            ray_dir = Gf.Vec3d(
                np.tan(fov_horizontal / 2) * ndc_x,
                np.tan(fov_vertical / 2) * ndc_y,
                -1
            ).GetNormalized()

            # Transform ray direction to world space
            world_ray_dir = camera_transform.TransformDir(ray_dir)

            # Create a ray
            ray = Gf.Ray(camera_position, world_ray_dir)

            # Intersect with selected prims
            hit_distance = far_clip

            for prim_path in selected_prims:
                prim = stage.GetPrimAtPath(prim_path)
                if prim.IsValid() and prim.IsA(UsdGeom.Boundable):
                    bbox = bbox_cache.ComputeWorldBound(prim)
                    intersects, distance = ray_box_intersection(ray, bbox)
                    if intersects and distance < hit_distance:
                        hit_distance = distance

            occlusion_buffer[y, x] = hit_distance

            # Increment pixel counter and check for early exit
            processed_pixels_count += 1
            if max_occl_culled_prims is not None and processed_pixels_count >= max_occl_culled_prims:
                return occlusion_buffer

    return occlusion_buffer

def is_occluded(bbox, camera_API, occlusion_buffer, occlusion_resolution):
    bbox_center = bbox.ComputeCentroid()
    
    # Transform bbox_center to camera space
    camera_space_point = camera_API.transform.GetInverse().Transform(bbox_center)
    
    # Manual projection to NDC space
    projection_matrix = camera_API.frustum.ComputeProjectionMatrix()
    projected_point = projection_matrix.Transform(camera_space_point)
     
    # Perform perspective division
    if len(projected_point) >= 4 and projected_point[3] != 0:
        ndc_point = Gf.Vec3d(
            projected_point[0] / projected_point[3],
            projected_point[1] / projected_point[3],
            projected_point[2] / projected_point[3]
        )
    else:
        return False  # Point is at infinity or invalid, consider it not occluded
       
    # Convert NDC to pixel coordinates
    pixel_x = int((ndc_point[0] + 1) * 0.5 * occlusion_resolution[0])
    pixel_y = int((ndc_point[1] + 1) * 0.5 * occlusion_resolution[1])
    
    # Check if the point is within the occlusion buffer
    if 0 <= pixel_x < occlusion_resolution[0] and 0 <= pixel_y < occlusion_resolution[1]:
        occlusion_depth = occlusion_buffer[pixel_y, pixel_x]
        return -camera_space_point[2] > occlusion_depth  # Note the negative sign
    
    return False

def expand_frustum(frustum, padding, far_clip_multiplier):
    # Widen the frustum by modifying the window attribute
    window = frustum.window
    window.min = Gf.Vec2d(window.min[0] - padding, window.min[1] - padding)
    window.max = Gf.Vec2d(window.max[0] + padding, window.max[1] + padding)
    frustum.window = window

    # Increase the far clipping plane
    near_far_range = frustum.nearFar
    near_clip = near_far_range.min
    far_clip = near_far_range.max * far_clip_multiplier
    frustum.nearFar = Gf.Range1d(near_clip, far_clip)

    return frustum

def create_frustum_curves(stage, camera_prim, time_code, padding, far_clip_multiplier):
    frustum_path = camera_prim.GetPath().AppendChild("frustum_visualization")
    frustum_curves = UsdGeom.BasisCurves.Define(stage, frustum_path)

    curveVertexCounts = [2, 2, 2, 2, 2, 2, 2, 2]
    frustum_curves.CreateCurveVertexCountsAttr().Set(curveVertexCounts)
    frustum_curves.CreateTypeAttr().Set(UsdGeom.Tokens.linear)
    frustum_curves.CreateWidthsAttr().Set([2])

    # Set the color to green for all curves
    green_color = Gf.Vec3f(0, 1, 0)  # RGB for green
    color_primvar = UsdGeom.PrimvarsAPI(frustum_curves).CreatePrimvar("displayColor", Sdf.ValueTypeNames.Color3fArray, UsdGeom.Tokens.constant)
    color_primvar.Set([green_color] * len(curveVertexCounts))

    camera = UsdGeom.Camera(camera_prim)

    # Get camera parameters
    focal_length = camera.GetFocalLengthAttr().Get(time_code)
    horizontal_aperture = camera.GetHorizontalApertureAttr().Get(time_code)
    vertical_aperture = camera.GetVerticalApertureAttr().Get(time_code)
    near_clip, far_clip = camera.GetClippingRangeAttr().Get(time_code)

    # Calculate field of view
    fov_horizontal = 2 * math.atan((horizontal_aperture * 0.5) / focal_length)
    fov_vertical = 2 * math.atan((vertical_aperture * 0.5) / focal_length)

    # Apply padding to field of view
    fov_horizontal += math.radians(padding)
    fov_vertical += math.radians(padding)

    # Expand far clip
    far_clip *= far_clip_multiplier

    # Calculate expanded frustum dimensions
    near_height = 2 * near_clip * math.tan(fov_vertical * 0.5)
    near_width = 2 * near_clip * math.tan(fov_horizontal * 0.5)
    far_height = 2 * far_clip * math.tan(fov_vertical * 0.5)
    far_width = 2 * far_clip * math.tan(fov_horizontal * 0.5)

    # Define frustum points in camera space
    local_points = [
        Gf.Vec3d(-near_width*0.5, -near_height*0.5, -near_clip),
        Gf.Vec3d(near_width*0.5, -near_height*0.5, -near_clip),
        Gf.Vec3d(near_width*0.5, near_height*0.5, -near_clip),
        Gf.Vec3d(-near_width*0.5, near_height*0.5, -near_clip),
        Gf.Vec3d(-far_width*0.5, -far_height*0.5, -far_clip),
        Gf.Vec3d(far_width*0.5, -far_height*0.5, -far_clip),
        Gf.Vec3d(far_width*0.5, far_height*0.5, -far_clip),
        Gf.Vec3d(-far_width*0.5, far_height*0.5, -far_clip)
    ]

    # Camera position in local space is always (0, 0, 0)
    camera_position = Gf.Vec3d(0, 0, 0)

    # Create frustum lines
    curve_points = []
    for point in local_points:
        curve_points.extend([camera_position, point])

    frustum_curves.CreatePointsAttr().Set(curve_points)

    # Debug output
    print(f"Camera local position: {camera_position}")
    print(f"Expanded frustum near point: {local_points[0]}")
    print(f"Expanded frustum far point: {local_points[4]}")

    return frustum_curves

def is_within_sphere(prim_bbox, camera_origin, sphere_radius):
    prim_center = prim_bbox.ComputeCentroid()
    distance = (prim_center - camera_origin).GetLength()
    return distance <= sphere_radius

def distance_based_culling(prim, camera_API, max_distance, bbox_cache, time_code, culling_method):
    prim_bbox = bbox_cache.ComputeWorldBound(prim)
    prim_center = prim_bbox.ComputeCentroid()
    camera_transform = camera_API.transform
    camera_pos = camera_transform.ExtractTranslation()
    camera_dir = camera_transform.ExtractRotationQuat().Transform(Gf.Vec3d(0, 0, -1))
    prim_to_camera = prim_center - camera_pos
    distance = Gf.Dot(prim_to_camera, camera_dir)
    
    # Cull the prim if the distance exceeds the max_distance threshold
    if distance > max_distance:
        if culling_method == "deactivate":
            prim.SetActive(False)
        else:  # culling_method == "invisible"
            imageable_type_API = UsdGeom.Imageable(prim)
            visibility_attr = imageable_type_API.GetVisibilityAttr()
            visibility_attr.Set(UsdGeom.Tokens.invisible, time_code)
        return True
    
    return False

node = hou.pwd()
stage = node.editableStage()
lops_selection_rule = "(/root/world/geo/set/** & %type:UsdGeomBoundable) - %descendants(%type:PointInstancer) + %type:PointInstancer + %instance"
ls = hou.LopSelectionRule()
ls.setPathPattern(lops_selection_rule)
selected_prims = ls.expandedPaths(stage=stage)

shot_cam = hou.pwd().parent().parm("camera_path").evalAsString()
far_clip_mult = hou.pwd().parent().parm("far_clip_mult").eval()
padding = hou.pwd().parent().parm("padding").eval()
camera_prim_path = Sdf.Path(shot_cam)
camera_prim = stage.GetPrimAtPath(camera_prim_path)
start_frame = int(hou.contextOption("rfstart"))
end_frame = int(hou.contextOption("rfend"))
exclusion = hou.pwd().parent().parm("exclusion").evalAsString()
enable_distance_culling = hou.pwd().parent().parm("enable_distance_culling").evalAsInt()
enable_frustum_culling = hou.pwd().parent().parm("enable_frustum_culling").evalAsInt()
culling_method = hou.pwd().parent().parm("culling_method").evalAsString()
visualize_frustum = hou.pwd().parent().parm("visualize_frustum").evalAsInt()
enable_spherical_exclusion = hou.pwd().parent().parm("enable_spherical_exclusion").evalAsInt()
enable_occlusion_culling = hou.pwd().parent().parm("enable_occlusion_culling").evalAsInt()
max_occl_culled_prims = hou.pwd().parent().parm("max_occl_culled_prims").evalAsInt()

if enable_spherical_exclusion:
    sphere_radius = hou.pwd().parent().parm("sphere_radius").eval()

if enable_distance_culling:
    max_distance = hou.pwd().parent().parm("max_distance").eval()

if exclusion:
    collection_name = exclusion
    collection_path = "/collections"
    collectionPrim = stage.GetPrimAtPath(collection_path)
    collection_api = Usd.CollectionAPI.Apply(collectionPrim, collection_name)
    collection_api.GetExpansionRuleAttr().Set(Usd.Tokens.explicitOnly)
    collection_query = collection_api.ComputeMembershipQuery()
    exclude_path = collection_api.ComputeIncludedPaths(collection_query, stage)
    exclude_prim_set = set(stage.GetPrimAtPath(path) for path in exclude_path)
else:
    exclude_prim_set = set()

if is_interactive_mode():
    step = (end_frame - start_frame) // 4
else:
    print("Farm mode active, sampling every 2 frames")
    step = 2
time_samples = list(range(start_frame, end_frame, step))

# Mode
mode = hou.pwd().parent().parm("mode").evalAsString()
data = {}

culling_results = {}
always_visible_prims = set()

occlusion_buffer = None
occlusion_resolution = (1024, 768)

culled_prims = 0
culled_instances = 0
culled_point_instances = 0
culled_native_prims = 0
occlusion_culled_prims = 0

for time_sample in time_samples:
    time_code = Usd.TimeCode(time_sample)
    camera_type_API = UsdGeom.Camera(camera_prim)
    camera_API = camera_type_API.GetCamera(time_code)
    frustum = camera_API.frustum
    expanded_frustum = expand_frustum(frustum, padding, far_clip_mult)
    
    bbox_cache = UsdGeom.BBoxCache(
        time_code,
        ["default", "render", "proxy", "guide"],
        useExtentsHint=True,
        ignoreVisibility=False
    )

    if enable_occlusion_culling:
        occlusion_buffer = create_occlusion_buffer(stage, camera_prim, occlusion_resolution, selected_prims, far_clip_mult, max_occl_culled_prims)

    # Get camera origin for spherical exclusion
    camera_transform = camera_API.transform
    camera_origin = camera_transform.ExtractTranslation()

    for prim_path in selected_prims:
        prim = stage.GetPrimAtPath(prim_path)

        if not prim:
            continue

        if not prim.IsActive():
            continue

        if exclude_prim_set and prim in exclude_prim_set:
            continue

        # Check for spherical exclusion
        if enable_spherical_exclusion:
            prim_bbox = bbox_cache.ComputeWorldBound(prim)
            if is_within_sphere(prim_bbox, camera_origin, sphere_radius):
                # Prim is within exclusion sphere
                if mode == "frame":
                    # Apply visibility immediately for frame mode
                    if culling_method == "invisible":
                        imageable_type_API = UsdGeom.Imageable(prim)
                        visibility_attr = imageable_type_API.GetVisibilityAttr()
                        visibility_attr.Set(UsdGeom.Tokens.inherited, time_code)
                    else:  # culling_method == "deactivate"
                        prim.SetActive(True)
                else:  # "average" or "any" mode
                    culling_results.setdefault(prim_path, []).append(True)
                continue


        # Perform distance-based culling if enabled
        if enable_distance_culling:
            if distance_based_culling(prim, camera_API, max_distance, bbox_cache, time_code, culling_method):
                culling_results.setdefault(prim_path, []).append(False)
                continue

        # Perform frustum culling if enabled
        if enable_frustum_culling:
            if prim.IsInstance():
                instance_xformable = UsdGeom.Xformable(prim)
                instance_transform = instance_xformable.ComputeLocalToWorldTransform(time_code)
                instance_untransformed_bbox = bbox_cache.ComputeUntransformedBound(prim)
                instance_bbox_range = instance_untransformed_bbox.ComputeAlignedRange()
                instance_bbox = Gf.BBox3d(instance_bbox_range)
                instance_bbox.Transform(instance_transform)
                is_visible = expanded_frustum.Intersects(instance_bbox)

                if not is_visible:
                    culled_instances += 1

                try:
                    if enable_occlusion_culling and is_visible:
                        is_visible = not is_occluded(instance_bbox, camera_API, occlusion_buffer, occlusion_resolution)
                except Exception as e:
                    print(f"Error in occlusion culling for prim {prim.GetPath()}: {str(e)}")
                    is_visible = True


                culling_results.setdefault(prim_path, []).append(is_visible)

                # Apply visibility immediately for frame mode
                if mode == "frame":
                    if is_visible:
                        prim.SetActive(True)
                    else:
                        if culling_method == "deactivate":
                            prim.SetActive(False)
                        else:  # culling_method == "invisible"
                            imageable_type_API = UsdGeom.Imageable(prim)
                            visibility_attr = imageable_type_API.GetVisibilityAttr()
                            visibility_attr.Set(UsdGeom.Tokens.invisible, time_code)

            elif prim.IsA(UsdGeom.PointInstancer):
                pointinstancer_type_API = UsdGeom.PointInstancer(prim)
                prototypes_rel = pointinstancer_type_API.GetPrototypesRel()
                
                # Check if prototype prims are valid
                prototype_prims = [stage.GetPrimAtPath(target) for target in prototypes_rel.GetTargets()]
                if any(not prototype_prim or not prototype_prim.IsValid() for prototype_prim in prototype_prims):
                    continue  # Skip this PointInstancer if any prototype is invalid

                protoIndices_attr = pointinstancer_type_API.GetProtoIndicesAttr()
                if not protoIndices_attr.HasValue():
                    continue
                protoIndices = protoIndices_attr.Get(time_sample)
                protoIndices_attr_len = len(protoIndices)
                num_prototypes = len(prototypes_rel.GetTargets())

                if any(proto_index >= num_prototypes for proto_index in protoIndices):
                    continue

                bboxes = bbox_cache.ComputePointInstanceWorldBounds(
                    pointinstancer_type_API, list(range(protoIndices_attr_len))
                )
                invisibleIds_attr_value = [idx for idx, bbox in enumerate(bboxes) if not expanded_frustum.Intersects(bbox)]
                invisibleIdsCount = len(invisibleIds_attr_value)

                if mode == "frame":
                    # Apply invisibility immediately for frame mode
                    pointinstancer_type_API.CreateInvisibleIdsAttr().Set(invisibleIds_attr_value, time_code)
                    visibility_attr = UsdGeom.Imageable(prim).GetVisibilityAttr()
                    visibility_attr.Set(UsdGeom.Tokens.inherited, time_code)
                else:  # For both "average" and "any" modes
                    # Store data for later processing
                    data.setdefault(prim_path, {"invisibleIds": [], "invisibleIdsCount": []})
                    data[prim_path]["invisibleIds"].append(invisibleIds_attr_value)
                    data[prim_path]["invisibleIdsCount"].append(invisibleIdsCount)

                continue

            else:
                bbox = bbox_cache.ComputeWorldBound(prim)
                is_visible = expanded_frustum.Intersects(bbox)

                try:
                    if enable_occlusion_culling and is_visible:
                        is_visible = not is_occluded(bbox, camera_API, occlusion_buffer, occlusion_resolution)
                except Exception as e:
                    print(f"Error in occlusion culling for prim {prim.GetPath()}: {str(e)}")
                    is_visible = True


                culling_results.setdefault(prim_path, []).append(is_visible)

    #Draw frustum
    if visualize_frustum:
        frustum_curves = create_frustum_curves(stage, camera_prim, time_code, padding, far_clip_mult)

# Apply Culling results for instances and regular prims
if mode in ["average", "any"]:
    for prim_path, visibility_list in culling_results.items():
        prim = stage.GetPrimAtPath(prim_path)
        if not prim or not prim.IsActive():
            continue

        # Skip prims that are always visible due to spherical exclusion
        if prim_path in always_visible_prims:
            continue

        if mode == "average":
            visible_frames = sum(visibility_list)
            total_frames = len(visibility_list)
            is_visible = visible_frames > 2  # Keep if visible in more than 2 frames
        elif mode == "any":
            is_visible = any(visibility_list)

        if is_visible:
            if culling_method == "invisible":
                imageable_type_API = UsdGeom.Imageable(prim)
                visibility_attr = imageable_type_API.GetVisibilityAttr()
                visibility_attr.Set(UsdGeom.Tokens.inherited)
            elif culling_method == "deactivate":
                prim.SetActive(True)
        else:
            if culling_method == "deactivate":
                prim.SetActive(False)
            else:  # culling_method == "invisible"
                imageable_type_API = UsdGeom.Imageable(prim)
                visibility_attr = imageable_type_API.GetVisibilityAttr()
                visibility_attr.Set(UsdGeom.Tokens.invisible)

# Applying averaged results for visibility
if mode in ["average", "any"] and data:
    for prim_path, visibility_data in data.items():
        prim = stage.GetPrimAtPath(prim_path)
        if not prim or not prim.IsActive():
            continue
        imageable_type_API = UsdGeom.Imageable(prim)
        visibility_attr = imageable_type_API.GetVisibilityAttr()

        # Handling for PointInstancers
        if "invisibleIds" in visibility_data:
            pointinstancer_type_API = UsdGeom.PointInstancer(prim)
            invisibleIds_average = set(np.arange(max(visibility_data['invisibleIdsCount'])))
            for invisibleIds in visibility_data.get("invisibleIds"):
                invisibleIds_average = invisibleIds_average.intersection(invisibleIds)
            invisibleIds_attr = pointinstancer_type_API.GetInvisibleIdsAttr()
            invisibleIds_attr.Set(np.array(sorted(invisibleIds_average)), time_code)
            visibility_attr.Set(UsdGeom.Tokens.inherited)
