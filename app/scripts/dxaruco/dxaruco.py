# -*- coding: utf-8 -*-
"""
Service containing logic for reading ARuco markers.

Created on Fri Jan 19 15:21:40 2018

@author: ekroeger, pfribourg, mcaniot
"""

import os
import sys
import math
import time
import functools

import json
import qi
import almath
import motion
import vision_definitions as vd # constants

# Special: for NAOqi OS (on Pepper and nao), pre-compiled cv2 is packaged
# TODO add check is_a_robot?

# patch path for local cv2 and numpy, should only be run when on robot
sys.path.insert(0, "scripts/dxaruco/_naoqios")
# Now we also need to rename a folder inside
if not os.path.exists("scripts/dxaruco/_naoqios/numpy/.libs"):
    # This means we haven't done the first setup
    import shutil
    # Pick the right version of numpy depending on how we're compiled
    if sys.maxunicode == 65535: # compiled with UCS2 Unicode
        shutil.move("scripts/dxaruco/_naoqios/_numpy_UCS2", "scripts/dxaruco/_naoqios/numpy")
    elif sys.maxunicode == 1114111: # compiled with UCS4 Unicode
        shutil.move("scripts/dxaruco/_naoqios/_numpy_UCS4", "scripts/dxaruco/_naoqios/numpy")
    else:
        assert False, "Unexpected sys.maxunicode: %i" % sys.maxunicode
    shutil.copytree("scripts/dxaruco/_naoqios/numpy/_libs",
                    "scripts/dxaruco/_naoqios/numpy/.libs")

import numpy
import cv2
import cv2.aruco

PACKAGE_UID = "dx-aruco"
SERVICE_NAME = "DXAruco"

CAMERA_DISTORTION_COEFF = numpy.array(
    [[0.13086823, -0.44239733, 0.0004841, -0.00322714, 0.16996254]])

CAMERA_MATRIX_RESOLUTION_2560_1920 = numpy.array([
    [2.41523736e+03, 0.00000000e+00, 1.25128063e+03],
    [0.00000000e+00, 2.41690366e+03, 9.94791007e+02]])

CAMERA_MATRIX_RESOLUTION_INDEPENDANT = numpy.array([
    [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]
])

CAMERA_RESOLUTIONS = [ vd.k16VGA, vd.k4VGA, vd.kVGA, vd.kQVGA, vd.kQQVGA, vd.kQQQVGA, vd.kQQQQVGA ]
K16VGA_RESOLUTION = {"x": 2560, "y": 1920}

CAMERA_DATAS_AT_RESOLUTION = { camera_resolution: {
        "matrix": numpy.append(CAMERA_MATRIX_RESOLUTION_2560_1920 / (2.**i), CAMERA_MATRIX_RESOLUTION_INDEPENDANT, axis=0),
        "image_size": (K16VGA_RESOLUTION["x"] / (2**i), K16VGA_RESOLUTION["y"] / (2**i)),
        "fps": 7 if i in [0, 1] else 10,
    }
    for i, camera_resolution in enumerate(CAMERA_RESOLUTIONS) }

CAMERAS = {
    vd.kTopCamera: "CameraTop",
    vd.kBottomCamera: "CameraBottom",
}

CAMERA_PARAMETERS = {
    "AutoExposition": 11,
    "AutoWhiteBalance": 12,
    "Brightness": 0,
    "Contrast": 1,
    "Saturation": 2,
    "Exposure": 17,
}

DEFAULT_PARAMS = {
    "camera": vd.kBottomCamera,
    "resolution": vd.k16VGA,
    "ids": list(), # ARuco ids, if empty, return all detected ids
    "size": 0.22, # ARuco real size in meters
    "color": list(), # ARuco real color in RGB, if empty not thresholding
    "dictionary": cv2.aruco.DICT_4X4_1000, # ARuco dictionary
    "position": "floor", # ARuco marker position "floor" or "wall",
    "effector_id": 2, # Track with bottom camera
    "asynchronous": False, # Default is synchronous
    "color_space_and_channels": [ vd.kYuvColorSpace, 1 ], # Default is gray level 1 channel
    "exposure": 400, # default exposure is 400, if 0, exposure not changed
    "try": 3, # default 3 try to detect ARuco (only apply for detect_with_try())
}

SUBSCRIBER_ID = "com-softbankrobotics-%s" % PACKAGE_UID

@qi.multiThreaded()
class Main(object):
    def __init__(self, application, logger):
        # qi
        self.application = application
        self.logger = logger
        self.session = self.application.session

        # services declaration
        self.services = [ "ALMotion", "ALMemory", "ALVideoDevice" ]

        # fetching mandatory services
        # self.logger.info("waiting & fetch services...")
        for service_name in self.services:
            # self.logger.info("waiting %s services..." % service_name)
            self.session.waitForService(service_name)
            setattr(self, service_name, self.session.service(service_name))
        # self.logger.info("waiting & fetch services services done")
        self.camera_id = 1
        # clean any previous subscribtion (on re-install or restart)
        self._unsubscribe_all()

        # local variables
        self.periodic_tasks = dict()

    def subscribe(self, _params):
        # self.logger.info("subscribe...")
        self.last_timestamp = 0
        self.image_nb = 0
        params = dict(DEFAULT_PARAMS) # copy default params
        params.update(_params)
        subscriber_id = self._subscribe(params)
        self.periodic_tasks[subscriber_id] = qi.PeriodicTask()
        self.periodic_tasks[subscriber_id].setCallback(functools.partial(self._task, subscriber_id, params))
        resolution = params["resolution"]
        fps = CAMERA_DATAS_AT_RESOLUTION[resolution]["fps"]
        self.periodic_tasks[subscriber_id].setUsPeriod(1000000 / fps)
        self.periodic_tasks[subscriber_id].start(True)
        # self.logger.info("subscribe done")
        return subscriber_id

    def get_home_position_in_world(self, params):
        p6Ds = self.detect_with_try_params_list(params)
        p2Ds_list = {}
        params_list = []
        if not isinstance(params, list):
            params_list.append(dict(DEFAULT_PARAMS))
        else:
            params_list = params

        id_list = []
        for tmp_params in params_list:
            id_list += tmp_params["ids"]

        for ids in id_list:
            try:
                p6D_world2target = p6Ds[ids]["world2target"]
                p2Ds_list[ids] = [p6D_world2target[0], p6D_world2target[1], p6D_world2target[-1]]
            except Exception as e:
                pass
        return p2Ds_list

    def get_home_position(self, params):
        p6Ds = self.detect_with_try_params_list(params)
        p6Ds_list = {}
        params_list = []
        if not isinstance(params, list):
            params_list.append(dict(DEFAULT_PARAMS))
        else:
            params_list = params

        id_list = []
        for tmp_params in params_list:
            id_list += tmp_params["ids"]
        for ids in id_list:
            try:
                p6D_world2target = almath.Position6D(p6Ds[ids]["world2target"])
                t_world2target = almath.transformFromPosition6D(p6D_world2target)
                self.logger.info("@@@ TARGET IN WORLD %s" % p6D_world2target)

                p2D_world2robot = almath.Pose2D(self.ALMotion.getRobotPosition(True))
                t_world2robot = almath.transformFromPose2D(p2D_world2robot)
                self.logger.info("@@@ ROBOT POSITION %s" % p2D_world2robot)

                t_robot2target = t_world2robot.inverse() * t_world2target
                p6D_robot2target = almath.position6DFromTransform(t_robot2target)
                self.logger.info("@@@ TARGET IN ROBOT %s" % p6D_robot2target)
                p6Ds_list[ids] = list(p6D_robot2target.toVector())
            except Exception as e:
                pass
        return p6Ds_list

    def detect_with_try_params_list(self, _params = [dict()]):
        # self.logger.info("detect...")
        params_list = []
        if isinstance(_params, list):
            for tmp_params in _params:
                default_params = dict(DEFAULT_PARAMS) # copy default params
                default_params.update(tmp_params)
                params_list.append(default_params)
        else:
            params_list.append(dict(DEFAULT_PARAMS))
        p6Ds = dict()
        for params in params_list:
            subscriber_id = self._subscribe(params)
            for i in range(params["try"]):
                try:
                    p6Ds = self.__task(subscriber_id, params)
                    self._unsubscribe(subscriber_id)
                    return p6Ds
                except Exception as e:
                    self.logger.warning("detect_with_try %s/%s failed: %s" % (i, params["try"], e))
                    #qi.async(lambda:None, delay=200000).wait()
            self._unsubscribe(subscriber_id)
        if not p6Ds:
            raise Exception("Can't detect ARuco marker")
        # self.logger.info("detect done")
        return p6Ds

    def detect_with_try(self, _params = dict()):
        # self.logger.info("detect...")
        params = dict(DEFAULT_PARAMS) # copy default params
        params.update(_params)
        subscriber_id = self._subscribe(params)
        p6Ds = dict()
        for i in range(params["try"]):
            try:
                p6Ds = self.__task(subscriber_id, params)
                break
            except Exception as e:
                self.logger.warning("detect_with_try %s/%s failed: %s" % (i, params["try"], e))
                qi.async(lambda:None, delay=200000).wait()
        self._unsubscribe(subscriber_id)
        if not p6Ds:
            raise Exception("Can't detect ARuco marker")
        # self.logger.info("detect done")
        return p6Ds

    def detect(self, _params = dict()):
        # self.logger.info("detect...")
        params = dict(DEFAULT_PARAMS) # copy default params
        params.update(_params)
        subscriber_id = self._subscribe(params)
        try:
            p6Ds = self.__task(subscriber_id, params)
        finally:
            self._unsubscribe(subscriber_id)
            # self.logger.info("detect done")
        return p6Ds

    def _subscribe(self, params):
        # self.logger.info("_subscribe...")
        camera = params["camera"]
        self.camera_id = params["camera"]
        resolution = params["resolution"]
        color_space, channels = params["color_space_and_channels"]
        params["dictionary"] = cv2.aruco.getPredefinedDictionary(params["dictionary"])
        fps = CAMERA_DATAS_AT_RESOLUTION[resolution]["fps"]
        subscriber_id = self.ALVideoDevice.subscribeCamera(SUBSCRIBER_ID,
                                                          camera,
                                                          resolution,
                                                          color_space,
                                                          fps)
        if params["exposure"]:
            actual_exposure = self.ALVideoDevice.getParameter(params["camera"], CAMERA_PARAMETERS["Exposure"])
            self.logger.info("actual exposure : " + repr(actual_exposure))
            if actual_exposure > params["exposure"]:
                self.logger.info("Change exposure for : " + repr(params["exposure"]))
                self.ALVideoDevice.setParameter(params["camera"], CAMERA_PARAMETERS["AutoExposition"], 0)
                self.ALVideoDevice.setParameter(params["camera"], CAMERA_PARAMETERS["Exposure"], params["exposure"])
        # qi.async(lambda:None, delay=1000000).wait()
        # self.logger.info("_subscribe done")
        return subscriber_id

    def get_default_parameters(self):
        return DEFAULT_PARAMS

    def unsubscribe(self, subscriber_id):
        # self.logger.info("unsubscribe %s..." % subscriber_id)
        self._unsubscribe(subscriber_id)
        self.periodic_tasks[subscriber_id].stop()
        del self.periodic_tasks[subscriber_id]
        # self.logger.info("unsubscribe %s done" % subscriber_id)

    def _unsubscribe(self, subscriber_id):
        # self.logger.info("_unsubscribe %s..." % subscriber_id)
        self.ALVideoDevice.unsubscribe(subscriber_id)
        self.ALVideoDevice.resetCamera(self.camera_id)
        # self.logger.info("_unsubscribe %s done" % subscriber_id)

    def __task(self, subscriber_id, params):
        # self.logger.info("__task %s..." % subscriber_id)
        if not params["asynchronous"]:
            # this is the default for detect() calls or synchronous subscribers
            p6Ds = self.___task(subscriber_id, params)
            # self.logger.info("__task %s done" % subscriber_id)
            return p6Ds
        else:
            # this is only used for async subscribers (never use with detect() calls)
            self.logger.info("@@@@@@ ASYNC TASK!")
            qi.async(self.___task, subscriber_id, params) \
                .then(functools.partial(self.___task_finished, subscriber_id, params))
        # self.logger.info("__task %s done" % subscriber_id)

    def ___task(self, subscriber_id, params):
        image, t_world2camera, timestamp = self._get_image_world2camera_and_timestamp(subscriber_id, params)
        p6Ds = self._detect_markers(params, image, t_world2camera, timestamp)
        return p6Ds

    def ___task_finished(self, subscriber_id, params, future):
        if future.hasError():
            self.logger.error("___task finished with error: %s" % future.error())
        elif future.hasValue():
            # self.logger.error("_on_detect_markers_finished finished with value")
            self._publish(subscriber_id, params, future.value())

    def _publish(self, subscriber_id, params, p6Ds):
        # self.logger.info("_publish %s..." % subscriber_id)
        self.image_nb += 1
        self.logger.info("@@@@@ IMAGE DONE: %s" % self.image_nb)
        for _id, p6D in p6Ds.iteritems():
            seconds, micro_seconds = p6D["timestamp"]
            timestamp = seconds*10e8 + micro_seconds*10e2
            event_value = {
                "id_aruco" : _id,
                "world2target" : p6D["world2target"],
                "timestamp" : p6D["timestamp"],
                "effector_id" : params["effector_id"]
            }
            if not params["asynchronous"] or \
                (params["asynchronous"] and timestamp > self.last_timestamp):
                self.last_timestamp = timestamp
                self.ALMemory.raiseEvent("DXAruco/%s/%s" % (subscriber_id, _id), json.dumps(event_value))
        # self.logger.info("_publish %s done" % subscriber_id)

    def _task(self, subscriber_id, params):
        # self.logger.info("_task %s..." % subscriber_id)
        if not params["asynchronous"]:
            # sync
            p6Ds = self.__task(subscriber_id, params)
            self._publish(subscriber_id, params, p6Ds)
        else:
            # async
            self.__task(subscriber_id, params)
        # self.logger.info("_task %s done" % subscriber_id)

    def _get_image_world2camera_and_timestamp(self, subscriber_id, params):
        t = time.time()
        # self.logger.info("_get_image_world2camera_and_timestamp %s..." % subscriber_id)
        image_remote = self.ALVideoDevice.getImageRemote(subscriber_id)
        if not image_remote:
            raise Exception("No data in image")
        camera = params["camera"]
        camera_name = CAMERAS[camera]
        seconds = image_remote[4]
        micro_seconds = image_remote[5]
        t_world2camera = almath.Transform(self.ALMotion._getSensorTransformAtTime(camera_name, seconds*10e8+micro_seconds*10e2))
        timestamp = [ seconds, micro_seconds ] # we store this for TrackEvent
        resolution = params["resolution"]
        x, y = CAMERA_DATAS_AT_RESOLUTION[resolution]["image_size"]
        color_space, channels = params["color_space_and_channels"]
        image = numpy.frombuffer(image_remote[6], dtype = numpy.uint8).reshape(y, x, channels)
        if params["color"] and color_space == vd.kBGRColorSpace:
            # self.logger.warning("Thresholding image...")
            lower_b = tuple( [ int(val) for val in params["color"] ] )
            upper_b = (255, 255, 255)
            image = cv2.inRange(image, lower_b, upper_b)
            # self.logger.warning("Thresholding image done")

        # cv2.imwrite("/home/nao/picture.png", image)
        d = time.time()-t
        self.logger.info("_get_image_world2camera_and_timestamp %s done %s" % (subscriber_id, d))
        return image, t_world2camera, timestamp

    def _unsubscribe_all(self):
        # self.logger.info("_unsubscribe_all...")
        for subscriber_id in self.ALVideoDevice.getSubscribers():
            if subscriber_id.startswith(SUBSCRIBER_ID):
                self.ALVideoDevice.unsubscribe(subscriber_id)
        # self.logger.info("_unsubscribe_all done")

    def _detect_markers(self, params, image, t_world2camera, timestamp):
        t = time.time()
        # self.logger.info("_detect_markers...")
        p6Ds = dict()
        corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(image, params["dictionary"])

        try:
            ids = [ int(_id) for _id in ids.flatten() ]
        except Exception as e:
            raise Exception("No markers found")

        # self.logger.info("@@@@@@@@@ IDs: %s" % ids)
        if ids:
            if params["ids"]:
                id_indices = [ indice for (indice, _id) in enumerate(ids) if _id in set(params["ids"]).intersection(set(ids)) ]
            else:
                id_indices = [ indice for (indice, _id) in enumerate(ids) ]

            # self.logger.info("@@@@@@@@@ IDs indices: %s" % id_indices)

            if id_indices:
                resolution = params["resolution"]
                rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, \
                                                                        params["size"], \
                                                                        CAMERA_DATAS_AT_RESOLUTION[resolution]["matrix"], \
                                                                        CAMERA_DISTORTION_COEFF)
                for indice in id_indices:
                    # switch from opencv coordinates to NAOqi coordinates
                    tvec = tvecs[indice][0] # translation vectors
                    x, y, z = tvec[2], -tvec[0], -tvec[1]
                    p3d_camera2target = almath.Position3D(x, y, z)

                    rvec = rvecs[indice][0] # rotation vectors
                    wx, wy, wz = rvec[2], -rvec[0], -rvec[1]
                    proj_rvec, _ = cv2.Rodrigues(numpy.array([wx, wy, wz]))

                    r_camera2target = almath.Rotation(proj_rvec.flatten())
                    t_camera2target = almath.transformFromRotationPosition3D(r_camera2target, p3d_camera2target)

                    if params["position"] == "floor":
                        r3d_correction = almath.Rotation3D(0., math.pi/2, 0.) # we have 90° on y axis that is invalid
                    elif params["position"] == "wall":
                        r3d_correction = almath.Rotation3D(0., math.pi, 0.) # we have 180° on y axis that is invalid

                    t_correction = almath.transformFromRotation3D(r3d_correction)
                    t_world2target = t_world2camera * t_camera2target * t_correction

                    p6D_world2target = almath.position6DFromTransform(t_world2target)
                    self.logger.info("@@@@@@@@@ ID: %s - P6D_WORLD2TARGET: %s" % (ids[indice], p6D_world2target))

                    p6Ds[ids[indice]] = {
                        "world2target": list(p6D_world2target.toVector()),
                        "timestamp": timestamp,
                    }
            else:
                raise Exception("No markers with IDs %s found" % params["ids"])

        d = time.time()-t
        self.logger.info("_detect_markers done %s" % d)
        return p6Ds

if __name__ == "__main__":
    application = qi.Application()
    application.start()

    logger = qi.getLogger(PACKAGE_UID)
    # mandatory to broadcast logs
    try:
        qicore_module = qi.module("qicore")
        log_provider = qicore_module.initializeLogging(application.session)
    except Exception as e:
        logger.warning("qicore module not found, logs won't be broadcasted")

    # instanciate, register & start the application
    try:
        lock_application = True
        logger.info("instanciation...")
        instance = Main(application, logger)
        logger.info("instanciation done, registration...")
        application.session.registerService(SERVICE_NAME, instance)
        logger.info("registration done")
    except Exception as e:
        logger.error("instanciation or registration finished with error: %s" %  e)
        lock_application = False

    if lock_application:
        logger.info("qi.Application() lock...")
        application.run()
        logger.info("qi.Application() lock released!")

    logger.info("bye bye")
    qi.async(lambda x: x, delay = 100000).wait() # need to broadcast log accross network