"""The only place that sends planning-scene changes to move_group (scene_core and legacy_scene call it).

Every change is ONE PlanningScene diff through /apply_planning_scene; reads use /get_planning_scene with
the component mask they need. The allowed-collision matrix is exchanged with scene_core as a plain dict
{"names": [...], "values": {(a, b): bool}, "defaults": {name: bool}}."""
from errors import err


def _pose_msg(p):
    from geometry_msgs.msg import Pose
    m = Pose()
    m.position.x, m.position.y, m.position.z = (float(v) for v in p["position"])
    m.orientation.x, m.orientation.y, m.orientation.z, m.orientation.w = (float(v) for v in p["orientation"])
    return m


class MoveItSceneIO:
    def __init__(self, ros):
        self.ros = ros
        self._mesh_msgs = {}            # (sha, scale) -> shape_msgs/Mesh

    def _get(self, components):
        from moveit_msgs.msg import PlanningSceneComponents
        from moveit_msgs.srv import GetPlanningScene
        req = GetPlanningScene.Request(components=PlanningSceneComponents(components=components))
        return self.ros.call(GetPlanningScene, "/get_planning_scene", req, timeout_s=10.0).scene

    def world_names(self):
        from moveit_msgs.msg import PlanningSceneComponents as C
        return [co.id for co in self._get(C.WORLD_OBJECT_NAMES).world.collision_objects]

    def attached_names(self):
        from moveit_msgs.msg import PlanningSceneComponents as C
        return [a.object.id for a in self._get(C.ROBOT_STATE_ATTACHED_OBJECTS).robot_state.attached_collision_objects]

    def world_poses(self, ids):
        from moveit_msgs.msg import PlanningSceneComponents as C
        out = {}
        for co in self._get(C.WORLD_OBJECT_GEOMETRY).world.collision_objects:
            if co.id in ids:
                p = co.pose
                out[co.id] = {"position": [p.position.x, p.position.y, p.position.z],
                              "orientation": [p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w]}
        return out

    def get_acm(self):
        from moveit_msgs.msg import PlanningSceneComponents as C
        m = self._get(C.ALLOWED_COLLISION_MATRIX).allowed_collision_matrix
        names = list(m.entry_names)
        vals = {(a, b): bool(m.entry_values[i].enabled[j]) for i, a in enumerate(names) for j, b in enumerate(names)}
        return {"names": names, "values": vals, "defaults": dict(zip(m.default_entry_names, m.default_entry_values))}

    @staticmethod
    def acm_msg(acm):
        from moveit_msgs.msg import AllowedCollisionEntry, AllowedCollisionMatrix
        m = AllowedCollisionMatrix()
        m.entry_names = list(acm["names"])
        m.entry_values = [AllowedCollisionEntry(enabled=[bool(acm["values"].get((a, b), False)) for b in acm["names"]])
                          for a in acm["names"]]
        m.default_entry_names = list(acm["defaults"].keys())
        m.default_entry_values = [bool(v) for v in acm["defaults"].values()]
        return m

    def get_padding(self):
        from moveit_msgs.msg import PlanningSceneComponents as C
        return {p.link_name: p.padding for p in self._get(C.LINK_PADDING_AND_SCALING).link_padding}

    def mesh_msg(self, sha, scale, mesh_cache):
        key = (sha, scale)
        if key not in self._mesh_msgs:
            from geometry_msgs.msg import Point
            from shape_msgs.msg import Mesh, MeshTriangle
            verts, faces = mesh_cache[sha]
            m = Mesh()
            m.vertices = [Point(x=float(x) * scale, y=float(y) * scale, z=float(z) * scale) for x, y, z in verts]
            m.triangles = [MeshTriangle(vertex_indices=[int(a), int(b), int(c)]) for a, b, c in faces]
            if len(self._mesh_msgs) > 256:
                self._mesh_msgs.clear()
            self._mesh_msgs[key] = m
        return self._mesh_msgs[key]

    def collision_object(self, obj, mesh_cache):
        from moveit_msgs.msg import CollisionObject
        from shape_msgs.msg import SolidPrimitive
        co = CollisionObject()
        co.id = obj["id"]
        co.header.frame_id = obj["frame_id"]
        co.pose = _pose_msg(obj["pose"])
        for s in obj["shapes"]:
            if s["type"] == "mesh":
                co.meshes.append(self.mesh_msg(s["sha256"], s["scale"], mesh_cache))
                co.mesh_poses.append(_pose_msg(s["pose"]))
                continue
            prim = SolidPrimitive()
            if s["type"] == "box":
                prim.type, prim.dimensions = SolidPrimitive.BOX, [float(v) for v in s["size"]]
            elif s["type"] == "sphere":
                prim.type, prim.dimensions = SolidPrimitive.SPHERE, [float(s["radius"])]
            else:       # MoveIt's order is [height, radius], the API's is radius, length
                prim.type, prim.dimensions = SolidPrimitive.CYLINDER, [float(s["length"]), float(s["radius"])]
            co.primitives.append(prim)
            co.primitive_poses.append(_pose_msg(s["pose"]))
        co.operation = CollisionObject.ADD
        return co

    def attached_object(self, obj, link, touch_links, mesh_cache):
        """AttachedCollisionObject with full geometry, ``obj`` posed in ``link`` (for virtual states)."""
        from moveit_msgs.msg import AttachedCollisionObject
        a = AttachedCollisionObject(link_name=link, touch_links=list(touch_links))
        a.object = self.collision_object(obj, mesh_cache)
        return a

    def apply(self, ops, mesh_cache, acm=None, padding=None, colors=None):
        """One diff. ops: ("remove", id, None) | ("add", id, obj) | ("attach", id, {"link", "touch_links"})
        (attaches the EXISTING world object) | ("detach", id, {"link"}) (puts it back into the world) |
        ("attach_full", id, {"obj", "link", "touch_links"}) (attach with geometry). MoveIt applies the robot
        state before the world, so detach + remove of one id work in one diff."""
        from moveit_msgs.msg import AttachedCollisionObject, CollisionObject, LinkPadding, ObjectColor, PlanningScene
        from moveit_msgs.srv import ApplyPlanningScene
        from std_msgs.msg import ColorRGBA
        ps = PlanningScene(is_diff=True)
        ps.robot_state.is_diff = True
        if acm is not None:
            ps.allowed_collision_matrix = self.acm_msg(acm)
        if padding:
            ps.link_padding = [LinkPadding(link_name=k, padding=float(v)) for k, v in sorted(padding.items())]
        if colors:
            ps.object_colors = [ObjectColor(id=k, color=ColorRGBA(r=float(v[0]), g=float(v[1]), b=float(v[2]),
                                                                  a=float(v[3])))
                                for k, v in sorted(colors.items())]
        for op, oid, arg in ops:
            if not oid:          # an empty id addresses (and a REMOVE wipes) every object
                raise ValueError("refusing a scene operation with an empty id")
            if op in ("attach", "detach", "attach_full"):
                if op == "attach_full":
                    aco = self.attached_object(arg["obj"], arg["link"], arg["touch_links"], mesh_cache)
                else:
                    aco = AttachedCollisionObject(link_name=arg["link"])
                    aco.object.id = oid
                    if op == "attach":
                        aco.touch_links = list(arg.get("touch_links") or [arg["link"]])
                        aco.object.operation = CollisionObject.ADD
                    else:
                        aco.object.operation = CollisionObject.REMOVE
                ps.robot_state.attached_collision_objects.append(aco)
                continue
            if op == "remove":
                co = CollisionObject(id=oid, operation=CollisionObject.REMOVE)
            else:
                co = self.collision_object(arg, mesh_cache)
            ps.world.collision_objects.append(co)
        try:
            return bool(self.ros.call(ApplyPlanningScene, "/apply_planning_scene", ApplyPlanningScene.Request(scene=ps),
                                      timeout_s=20.0).success)
        except Exception as e:     # noqa: BLE001
            raise err(504, "apply_unconfirmed", f"apply_planning_scene failed: {getattr(e, 'msg', e)}")
