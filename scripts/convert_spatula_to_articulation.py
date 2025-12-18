#!/usr/bin/env python3
# Copyright (c) 2024, lekisaac Project
# SPDX-License-Identifier: Apache-2.0

"""Convert Spatula USD from single RigidObject to Articulation with Fixed Joint.

This script creates a new Spatula USD where Handle and Blade are separate rigid bodies
connected by a fixed joint, allowing different friction settings per body.

Structure:
- /Root (Xform with ArticulationRootAPI)
  - /Root/Handle (Xform with RigidBodyAPI) - high friction for grasping
    - /Root/Handle/SpatulaHandle (Mesh with CollisionAPI)
  - /Root/Blade (Xform with RigidBodyAPI) - low friction for sliding
    - /Root/Blade/SpatulaBlade (Mesh with CollisionAPI)
  - /Root/HandleToBladeJoint (FixedJoint connecting Handle and Blade)
"""

from pxr import Usd, UsdGeom, UsdPhysics, UsdShade, Sdf, Gf


def convert_spatula_to_articulation(input_path: str, output_path: str):
    """Convert spatula from RigidObject to Articulation with fixed joint."""

    # Open source stage
    src_stage = Usd.Stage.Open(input_path)

    # Create new stage
    dst_stage = Usd.Stage.CreateNew(output_path)
    dst_stage.SetDefaultPrim(dst_stage.DefinePrim("/Root", "Xform"))

    # Set up meters per unit
    UsdGeom.SetStageUpAxis(dst_stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(dst_stage, 1.0)

    # Get source meshes
    handle_mesh_src = src_stage.GetPrimAtPath("/Root/Spatula_08/Spatula_08/SpatulaHandle")
    blade_mesh_src = src_stage.GetPrimAtPath("/Root/Spatula_08/Spatula_08/SpatulaBlade")
    material_src = src_stage.GetPrimAtPath("/Root/Spatula_08/Looks/Spatula08_Mat")

    # Create root with ArticulationRootAPI
    root = dst_stage.GetPrimAtPath("/Root")
    UsdPhysics.ArticulationRootAPI.Apply(root)

    # Create Handle body (root link of articulation)
    handle_xform = UsdGeom.Xform.Define(dst_stage, "/Root/Handle")
    handle_prim = handle_xform.GetPrim()

    # Apply RigidBodyAPI to Handle
    rigid_api = UsdPhysics.RigidBodyAPI.Apply(handle_prim)
    mass_api = UsdPhysics.MassAPI.Apply(handle_prim)
    mass_api.CreateMassAttr().Set(0.005)  # 5g for handle

    # Copy Handle mesh
    handle_mesh_dst = UsdGeom.Mesh.Define(dst_stage, "/Root/Handle/SpatulaHandle")
    _copy_mesh(handle_mesh_src, handle_mesh_dst.GetPrim())

    # Apply CollisionAPI to Handle mesh
    UsdPhysics.CollisionAPI.Apply(handle_mesh_dst.GetPrim())

    # Create Blade body (child link)
    blade_xform = UsdGeom.Xform.Define(dst_stage, "/Root/Blade")
    blade_prim = blade_xform.GetPrim()

    # Apply RigidBodyAPI to Blade
    rigid_api = UsdPhysics.RigidBodyAPI.Apply(blade_prim)
    mass_api = UsdPhysics.MassAPI.Apply(blade_prim)
    mass_api.CreateMassAttr().Set(0.005)  # 5g for blade

    # Copy Blade mesh
    blade_mesh_dst = UsdGeom.Mesh.Define(dst_stage, "/Root/Blade/SpatulaBlade")
    _copy_mesh(blade_mesh_src, blade_mesh_dst.GetPrim())

    # Apply CollisionAPI to Blade mesh
    UsdPhysics.CollisionAPI.Apply(blade_mesh_dst.GetPrim())

    # Create Fixed Joint connecting Handle and Blade
    joint = UsdPhysics.FixedJoint.Define(dst_stage, "/Root/HandleToBladeJoint")

    # Set joint bodies (body0 = Handle, body1 = Blade)
    joint.CreateBody0Rel().SetTargets([Sdf.Path("/Root/Handle")])
    joint.CreateBody1Rel().SetTargets([Sdf.Path("/Root/Blade")])

    # Set joint local poses - both at origin for stability
    # PhysX fixed joints work best when local poses are at body origins
    joint.CreateLocalPos0Attr().Set(Gf.Vec3f(0, 0, 0))
    joint.CreateLocalPos1Attr().Set(Gf.Vec3f(0, 0, 0))
    joint.CreateLocalRot0Attr().Set(Gf.Quatf(1, 0, 0, 0))
    joint.CreateLocalRot1Attr().Set(Gf.Quatf(1, 0, 0, 0))

    # Copy material
    _copy_material(src_stage, dst_stage, "/Root/Spatula_08/Looks/Spatula08_Mat", "/Root/Looks/SpatulaMat")

    # Bind material to meshes
    mat_prim = dst_stage.GetPrimAtPath("/Root/Looks/SpatulaMat")
    if mat_prim:
        material = UsdShade.Material(mat_prim)
        UsdShade.MaterialBindingAPI.Apply(handle_mesh_dst.GetPrim()).Bind(material)
        UsdShade.MaterialBindingAPI.Apply(blade_mesh_dst.GetPrim()).Bind(material)

    # Create physics materials for each body
    _create_physics_material(dst_stage, "/Root/HandlePhysicsMaterial", 5.0, 5.0, 0.0)
    _create_physics_material(dst_stage, "/Root/BladePhysicsMaterial", 0.0, 0.0, 0.0)

    # Bind physics materials
    handle_phys_mat = UsdShade.Material(dst_stage.GetPrimAtPath("/Root/HandlePhysicsMaterial"))
    blade_phys_mat = UsdShade.Material(dst_stage.GetPrimAtPath("/Root/BladePhysicsMaterial"))

    UsdShade.MaterialBindingAPI.Apply(handle_mesh_dst.GetPrim()).Bind(
        handle_phys_mat, UsdShade.Tokens.weakerThanDescendants, "physics"
    )
    UsdShade.MaterialBindingAPI.Apply(blade_mesh_dst.GetPrim()).Bind(
        blade_phys_mat, UsdShade.Tokens.weakerThanDescendants, "physics"
    )

    # Save
    dst_stage.GetRootLayer().Save()
    print(f"Saved articulation spatula to: {output_path}")

    # Print structure
    print("\n=== New Structure ===")
    for prim in dst_stage.Traverse():
        indent = "  " * (len(str(prim.GetPath()).split('/')) - 2)
        apis = []
        if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
            apis.append("ArticulationRoot")
        if prim.HasAPI(UsdPhysics.RigidBodyAPI):
            apis.append("RigidBody")
        if prim.HasAPI(UsdPhysics.CollisionAPI):
            apis.append("Collision")
        api_str = f" [{', '.join(apis)}]" if apis else ""
        print(f"{indent}{prim.GetPath().name} ({prim.GetTypeName()}){api_str}")


def _copy_mesh(src_prim, dst_prim):
    """Copy mesh data from source to destination."""
    src_mesh = UsdGeom.Mesh(src_prim)
    dst_mesh = UsdGeom.Mesh(dst_prim)

    # Copy geometry attributes
    if src_mesh.GetPointsAttr().Get():
        dst_mesh.CreatePointsAttr().Set(src_mesh.GetPointsAttr().Get())
    if src_mesh.GetFaceVertexCountsAttr().Get():
        dst_mesh.CreateFaceVertexCountsAttr().Set(src_mesh.GetFaceVertexCountsAttr().Get())
    if src_mesh.GetFaceVertexIndicesAttr().Get():
        dst_mesh.CreateFaceVertexIndicesAttr().Set(src_mesh.GetFaceVertexIndicesAttr().Get())
    if src_mesh.GetNormalsAttr().Get():
        dst_mesh.CreateNormalsAttr().Set(src_mesh.GetNormalsAttr().Get())
        dst_mesh.SetNormalsInterpolation(src_mesh.GetNormalsInterpolation())
    if src_mesh.GetExtentAttr().Get():
        dst_mesh.CreateExtentAttr().Set(src_mesh.GetExtentAttr().Get())

    # Copy primvars (UVs, etc.)
    src_primvars = UsdGeom.PrimvarsAPI(src_prim)
    dst_primvars = UsdGeom.PrimvarsAPI(dst_prim)
    for primvar in src_primvars.GetPrimvars():
        name = primvar.GetPrimvarName()
        if primvar.Get() is not None:
            new_primvar = dst_primvars.CreatePrimvar(name, primvar.GetTypeName(), primvar.GetInterpolation())
            new_primvar.Set(primvar.Get())
            if primvar.GetIndices():
                new_primvar.SetIndices(primvar.GetIndices())


def _copy_material(src_stage, dst_stage, src_path, dst_path):
    """Copy material (simplified - just create basic material)."""
    scope_path = "/".join(dst_path.split("/")[:-1])
    if not dst_stage.GetPrimAtPath(scope_path):
        UsdGeom.Scope.Define(dst_stage, scope_path)

    material = UsdShade.Material.Define(dst_stage, dst_path)
    shader = UsdShade.Shader.Define(dst_stage, dst_path + "/Shader")
    shader.CreateIdAttr("UsdPreviewSurface")
    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(0.5, 0.5, 0.5))
    material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")


def _create_physics_material(stage, path, static_friction, dynamic_friction, restitution):
    """Create a physics material."""
    material = UsdShade.Material.Define(stage, path)
    physics_api = UsdPhysics.MaterialAPI.Apply(material.GetPrim())
    physics_api.CreateStaticFrictionAttr().Set(static_friction)
    physics_api.CreateDynamicFrictionAttr().Set(dynamic_friction)
    physics_api.CreateRestitutionAttr().Set(restitution)


if __name__ == "__main__":
    import os

    script_dir = os.path.dirname(os.path.abspath(__file__))
    assets_dir = os.path.join(script_dir, "..", "assets", "objects")

    input_path = os.path.join(assets_dir, "Spatula.usd")
    output_path = os.path.join(assets_dir, "Spatula_articulation.usd")

    convert_spatula_to_articulation(input_path, output_path)
