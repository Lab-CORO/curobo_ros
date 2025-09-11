from abc import ABC, abstractmethod
from curobo.types.camera import CameraObservation
import torch


class CameraStrategy:
    '''
    This class is a strategie paterne to implement the joint speed control. 
    It has a timer called every dt to send the next speed command
    '''
    @abstractmethod
    def __init__(self, node, world_model):
        self.world_model = world_model
        # update the voxel obstacle at 30hz
        camera_update_hz = 30
        self.timer_update_camera = self.create_timer(1/camera_update_hz, self.update_camera)
        pass


    @abstractmethod
    def get_depth_map(self):
        pass


    @abstractmethod
    def get_camera_intrinsic(self):
        pass
        
    def update_camera(self):
        '''
        This method get the camera depth map and add it in the collision map.
        TODO using a point cloud for robot segmentation with multi-cameras
        '''
        if (self.get_depth_map() is None):
            return
        # get depth map from camera strategy
        self.world_model.decay_layer("world")
        data_camera = CameraObservation(
            depth_image=self.get_depth_map()/1000, 
            intrinsics=self.get_camera_intrinsic(), 
            pose=self.camera_pose)

        data_camera = data_camera.to(device=self.tensor_args.device)
        self.world_model.add_camera_frame(data_camera, "world")
        self.world_model.process_camera_frames("world", False)
        torch.cuda.synchronize()
        self.world_model.update_blox_hashes()
        # self.debug_voxel()


    # def debug_voxel(self):
    #     '''
    #     This method show voxels with rviz. 
    #     TODO change it and using voxelmap method
    #     '''
    #     # Voxel debug
    #     bounding = Cuboid("t", dims=[2.0, 2.0, 2.0], pose=[
    #                       0.0, 0.0, 1.0, 1.0, 0.0, 0.0, 0.0])

    #     voxel_size = self.get_parameter(
    #         'voxel_size').get_parameter_value().double_value

    #     voxels = self.world_model.get_voxels_in_bounding_box(
    #         bounding, voxel_size)

    #     if voxels.shape[0] > 0:
    #         voxels = voxels.cpu().numpy()
    #     self.marker_publisher.publish_markers_voxel(voxels, voxel_size)
