import unittest
import gpt5
import matplotlib.pyplot as plt
import numpy as np
import cv2
import yaml

class MyTestCase(unittest.TestCase):
    def test_occupancy_empty(self):
        scene = gpt5.Scene()
        #seed = random.randint(0, 1000)
        resolution = 2.0
        area = (40.0, 20.0)
        threshold = 0.0
        #scale = 0.4
        scale = 0.00
        seed = 41
        (res, occ_map) = scene.generate_perlin_navigable_zone(area_size=area,
                                                              resolution=resolution,
                                                              threshold=threshold,
                                                              scale=scale,
                                                              seed=seed)

        robot_start_xy, robot_stop_xy = gpt5.get_reachable_locations(res, occ_map)

        gpt5.export_sdf(scene, "scene_empty")
        gpt5.export_usda(scene, "scene_empty")
        gpt5.export_metadata(seed, threshold, resolution, area, scale, robot_start_xy, robot_stop_xy, occ_map, "scene_empty")

        self.assertTrue(np.array_equal(np.array([-10.0, 0.0]), robot_start_xy)) # add assertion here

        # retrieve robot_xy
        with open("scene_empty.yml", 'r') as file:
            data = yaml.safe_load(file)
        robot_xy = np.array(data['robot_start_xy'])

        self.assertTrue(np.array_equal(np.array([-10.0, 0.0]), robot_xy))  # add assertion here

    def test_occupancy_stuff(self):
        scene = gpt5.Scene()
        #seed = random.randint(0, 1000)
        resolution = 2.0
        area = (40.0, 20.0)
        threshold = 0.0
        scale = 0.4
        seed = 41
        (res, occ_map) = scene.generate_perlin_navigable_zone(area_size=area,
                                                              resolution=resolution,
                                                              threshold=threshold,
                                                              scale=scale,
                                                              seed=seed)

        robot_start_xy, robot_stop_xy = gpt5.get_reachable_locations(res, occ_map)

        gpt5.export_sdf(scene, "scene_stuff")
        gpt5.export_usda(scene, "scene_stuff")
        gpt5.export_metadata(seed, threshold, resolution, area, scale, robot_start_xy, robot_stop_xy, occ_map, "scene_stuff")

        self.assertTrue(np.array_equal(np.array([-2.0, 0.0]), robot_start_xy)) # add assertion here

        # retrieve robot_xy
        with open("scene_stuff.yml", 'r') as file:
            data = yaml.safe_load(file)
        robot_xy = np.array(data['robot_start_xy'])

        self.assertTrue(np.array_equal(np.array([-2.0, 0.0]), robot_xy))  # add assertion here

    def test_sdf(self):
        scene = gpt5.Scene()
        #seed = random.randint(0, 1000)
        seed = 42
        scene.generate_perlin_navigable_zone(area_size=(20.0, 20.0), resolution=2.0, threshold=0.0, scale=0.4, seed=seed)

        gpt5.export_sdf(scene, "scene")
        #self.assertEqual(True, False)  # add assertion here

    def test_usd(self):
        scene = gpt5.Scene()
        #seed = random.randint(0, 1000)
        seed = 42
        scene.generate_perlin_navigable_zone(area_size=(20.0, 20.0), resolution=2.0, threshold=0.0, scale=0.4, seed=seed)

        gpt5.export_usda(scene, "scene")
        #self.assertEqual(True, False)  # add assertion here

    def test_python(self):
        print((20.0//4.0) * 4.0)
        tmp = np.array([1,2,3])
        print(tmp+1)
        tmp = np.array([[1,2,3],[4,5,6]])
        print(np.max(tmp))

if __name__ == '__main__':
    unittest.main()
