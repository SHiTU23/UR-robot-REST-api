"""Read a mesh file in a throwaway process (STL, OBJ, PLY, DAE via trimesh).

A malformed upload can crash or hang a mesh loader, and the REST server must
survive any file a client stores, so meshes are parsed by running this file:

    python3 mesh_parse.py <mesh file> <out.npz>

It writes vertices (N x 3 float64) and faces (M x 3 int64) with every node
transform applied (all meshes in the file merged). Exit 0 on success; exit 2
with a one-line reason on stderr for a file that is readable but unusable.
"""
import sys

import numpy as np


def load(path):
    import trimesh
    obj = trimesh.load(path, force="scene", process=False)
    meshes = []
    if isinstance(obj, trimesh.Scene):
        for node in obj.graph.nodes_geometry:
            T, name = obj.graph[node]
            g = obj.geometry[name]
            if isinstance(g, trimesh.Trimesh) and len(g.faces):
                m = g.copy()
                m.apply_transform(T)
                meshes.append(m)
    elif isinstance(obj, trimesh.Trimesh) and len(obj.faces):
        meshes.append(obj)
    if not meshes:
        raise ValueError("no triangles found in the file")
    verts, faces, off = [], [], 0
    for m in meshes:
        verts.append(np.asarray(m.vertices, float))
        faces.append(np.asarray(m.faces, np.int64) + off)
        off += len(m.vertices)
    return np.vstack(verts), np.vstack(faces)


def main(path, out):
    try:
        v, f = load(path)
    except Exception as e:          # noqa: BLE001 -- any loader failure is "unreadable"
        print(f"not a readable mesh: {type(e).__name__}: {e}".splitlines()[0], file=sys.stderr)
        return 2
    np.savez(out, vertices=v, faces=f)
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1], sys.argv[2]))
