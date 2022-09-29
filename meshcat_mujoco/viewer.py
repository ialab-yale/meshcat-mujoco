import meshcat
from meshcat import Visualizer
import meshcat.geometry as mc_geom
import meshcat.transformations as mc_trans
from dm_control.mjcf import parser
from dm_control.mjcf import constants
from dm_control.mjcf.traversal_utils import commit_defaults
import mujoco 
import numpy as np
import tempfile
from pathlib import Path
import random


_DEFAULT_GEOM_TYPE = "sphere"

def get_rand_color():
    random_number = random.randint(0,16777215)
    hex_number = str(hex(random_number))
    hex_number ='0x'+ hex_number[2:]
    return hex_number

def _parse_mesh(_body_viz, _geom, _mesh_dir, _xml_tree):
    if hasattr(_geom.mesh, "file"):
        _mesh_file = _geom.mesh.file 
    else: 
        for _mesh_asset in _xml_tree.asset.mesh:
            if _mesh_asset.name == _geom.mesh.name:
                _mesh_file = _mesh_asset.file
    _name       = _mesh_file.get_vfs_filename().split('-')[0]
    _ext        = _mesh_file.extension
    _mesh_loc   = _mesh_dir + _name + _ext
    if _ext == '.stl':
        geom_constr = mc_geom.StlMeshGeometry
    elif _ext == '.obj':
        geom_constr = mc_geom.ObjMeshGeometry

    _geom_viz = _body_viz[_geom.mesh.name]
    _geom_viz.set_object(
        geom_constr.from_file(
            _mesh_loc
        ),
        mc_geom.MeshLambertMaterial(
                color=get_rand_color(),#0xff22dd,
                reflectivity=0.8)
    )
    geom_pos = np.zeros(3)
    geom_quat = np.zeros(4)
    geom_quat[0] = 1.0
    if _geom.pos is not None:
        geom_pos[:] = _geom.pos
    if _geom.quat is not None:
        geom_quat[:] = _geom.quat[:]
    tf = mc_trans.quaternion_matrix(geom_quat)
    tf[:3,3]  = geom_pos
    _geom_viz.set_transform(tf)
    return _geom_viz

def _parse_sphere(_body_viz, _geom, _xml_tree):
    if _geom.size is None:
        commit_defaults(_geom)
    size = _geom.size 
    radius = size[0]
    geom_pos = np.zeros(3)
    geom_quat = np.zeros(4)
    geom_quat[0] = 1.0
    if _geom.pos is not None:
        geom_pos[:] = _geom.pos
    if _geom.quat is not None:
        geom_quat[:] = _geom.quat[:]
    tf = mc_trans.quaternion_matrix(geom_quat)
    tf[:3,3]  = geom_pos
    _geom_viz = _body_viz[_geom.name]

    _geom_viz.set_object(
        mc_geom.Sphere(radius),
        mc_geom.MeshLambertMaterial(
                color=get_rand_color(),#0x6495ed,
                opacity=0.5,
                reflectivity=0.8)
    )
    _geom_viz.set_transform(tf)
    return _geom_viz


class MeshCatVisualizer(Visualizer):
    def __init__(self, xml_path, mj_model, mj_data) -> None:
        Visualizer.__init__(self)
        self._mj_data = mj_data
        self._mj_model = mj_model
        self._xml_path = xml_path
        _xml_root = '/'.join(self._xml_path.split('/')[:-1])+'/.'


        self._xml_tree = parser.from_path(xml_path)
        self._mesh_dir = _xml_root + '/' + self._xml_tree.compiler.meshdir

        self._body_names = []
        self._site_names = []
        self._add_body(self._xml_tree.worldbody, None)

        self.is_alive = True
        self.open()
        
    def _add_body(self, body, parent_body):
        body_idx = len(self._body_names)
        if not parent_body:
            body_name = constants.WORLDBODY
        else:
            body_name = body.name if body.name else f"body{body_idx}"
        
        geoms = body.geom if hasattr(body, "geom") else []
        sites = body.site if hasattr(body, "site") else []
        if geoms: 
            if body_name == constants.WORLDBODY:
                if geoms[0].name:
                    body_name = geoms[0].name
                else:
                    if geoms[0].type == "plane":
                        body_name = "ground"

        _body_viz   = self[body_name]
        if body_name is not constants.WORLDBODY:
            self._body_names.append(body_name)
            body_id     = mujoco.mj_name2id(self._mj_model, mujoco.mjtObj.mjOBJ_BODY, body_name)
            body_pos    = self._mj_data.xpos[body_id]
            body_quat   = self._mj_data.xquat[body_id]
            tf          = mc_trans.quaternion_matrix(body_quat)
            tf[:3,3]    = body_pos
            _body_viz.set_transform(tf)

                        
        for geom in geoms:
            if geom.type is None:
                commit_defaults(geom, "type")
                # _geom_type = geom.type
                if geom.type is None:
                    geom.type = _DEFAULT_GEOM_TYPE
            # else:
            #     _geom_type = geom.dclass.dclass
            if geom.dclass:
                if geom.dclass.dclass == "visual":
                    if (geom.type == constants.MESH):# or (_geom_type =='visual'):
                        _parse_mesh(_body_viz, geom, self._mesh_dir, self._xml_tree)
                    if (geom.type == "sphere"):
                        _parse_sphere(_body_viz, geom, self._xml_tree)
            else:
                    if (geom.type == constants.MESH):# or (_geom_type =='visual'):
                        _parse_mesh(_body_viz, geom, self._mesh_dir, self._xml_tree)
                    # if (geom.type == "sphere"):
                    #     _parse_sphere(_body_viz, geom, self._xml_tree)
        
        for site in sites:
            self._site_names.append(site.name)
            if site.type == "sphere":
                _parse_sphere(self, site, self._xml_tree)

        # Recurse.
        for child_body in body.body:
            self._add_body(child_body, body)
    def render(self):
        for body_name in self._body_names:
            if body_name != "floor":
                body_id = mujoco.mj_name2id(self._mj_model, mujoco.mjtObj.mjOBJ_BODY, body_name)
                body_xpos = self._mj_data.xpos[body_id]
                body_xmat = self._mj_data.xmat[body_id].reshape((3,3))            
                tf = np.eye(4)
                tf[:3,:3]   = body_xmat
                tf[:3,3]    = body_xpos
                self[body_name].set_transform(tf)
        for site_name in self._site_names:
            site_id = mujoco.mj_name2id(self._mj_model, mujoco.mjtObj.mjOBJ_SITE, site_name)
            site_xpos = self._mj_data.site_xpos[site_id]
            site_xmat = self._mj_data.site_xmat[site_id].reshape((3,3))
            tf = np.eye(4)
            tf[:3,:3]   = site_xmat
            tf[:3,3]    = site_xpos
            self[site_name].set_transform(tf)

    def close(self):
        pass