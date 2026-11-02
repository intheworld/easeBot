"""Online collision queries, independent of the offline simulation backend."""

def build_scene(urdf, plan, scene, *, targets=None):
    from .fcl_scene import FCLScene
    model = FCLScene(urdf, plan, scene)
    return model, model.data
