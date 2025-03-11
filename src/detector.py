import pyrealsense2 as rs
import numpy as np
import cv2
import torch
import time

from ultralytics import YOLO
from openni import openni2
# from pyKinectAzure.pykinect_azure import pykinect
import pykinect_azure as pykinect
from pykinect_azure import K4A_CALIBRATION_TYPE_COLOR, K4A_CALIBRATION_TYPE_DEPTH, k4a_float2_t, K4A_FRAMES_PER_SECOND_30, K4A_WIRED_SYNC_MODE_STANDALONE


class YoloResult:
    def __init__(self,name, box, x, y, conf) -> None:
        self.name = name
        self.box = box
        self.x = x
        self.y = y
        self.conf = conf

    def __str__(self):
        return f'name:{self.name},box:{self.box},x:{self.x},y:{self.y}'


class Detector:

    def __init__(self) -> None:
        self.model = YOLO('./model/yolo11m.pt')
        # 检查是否有可用的GPU
        self.device = 'cuda' if torch.cuda.is_available() else 'cpu'
        print(f"Using device: {self.device}")
        # 将模型加载到GPU
        self.model.to(self.device)

        # self.K_kinect = np.array([614.86962890625, 0.0, 635.5834350585938,
        #                         0.0, 614.7677612304688, 364.99200439453125,
        #                         0.0, 0.0, 1.0]).reshape(3,3)  # 720P

        # self.K_kinect = np.array([922.304443359375, 0.0, 953.6251220703125,
        #                           0.0, 922.151611328125,547.738037109375,
        #                           0.0, 0.0, 1.0]).reshape(3,3)  # 1080P

        self.K_kinect = np.array([926.658159, 0.000000, 969.015082,
                                    0.000000, 927.187798, 549.125794,
                                    0.000000, 0.000000, 1.000000]).reshape(3,3)  # 1080P
        
        # self.K_kinect = np.array([930.4957055212508, 0.0, 955.3281992157035, 
        #                           0.0, 929.1382318220207, 557.7992310028158, 
        #                           0.0, 0.0, 1.0]).reshape(3,3)  # 1080P


        self.K_realsense =  np.array([605.94,0,320.65,
                                      0,605.61,241.75,
                                      0,0,1.0]).reshape(3,3)
        
        self.K_astra = np.array([517.2266360075698, 0, 325.8070672045541, 
                                 0, 517.0432548089038, 240.7548229951128, 
                                 0, 0, 1]).reshape(3,3)


    def detect(self,camera='cam',pattern='realtime',target='person',depth=False,range=0.5):
        """
            检测主函数
            'camera':使用的相机设备，'k4a'或'kinect'表示使用微软kinect相机；
                                'realsense'表示使用英特尔realsense相机；
                                'astra'表示使用奥比中光Astra S相机；
                                'cam'或其他表示使用电脑自带摄像头。
            'pattern':检测模式，有三种模式：'realtime'表示实时检测，按q键停止；
                                        'find'表示寻找目标，当找到目标并且目标的置信度较高时停止检测；
                                        其他模式
        """
        point = (0,0)
        results_detect = []
        if camera=='k4a' or camera=='kinect':
            # kinect_module = r'/usr/lib/x86_64-linux-gnu/libk4a.so'
            # self.k4a = pykinect(kinect_module)
            # self.k4a.device_open()
            # device_config = self.k4a.config
            # device_config.color_resolution = self.k4a.K4A_COLOR_RESOLUTION_1080P

            pykinect.initialize_libraries()
            device_config = pykinect.default_configuration
            device_config.color_format = pykinect.K4A_IMAGE_FORMAT_COLOR_MJPG
            device_config.color_resolution = pykinect.K4A_COLOR_RESOLUTION_1080P  # 1080P:1920x1080, 720P:1280x720
            device_config.depth_mode = pykinect.K4A_DEPTH_MODE_WFOV_2X2BINNED
            # device_config.depth_mode = pykinect.K4A_DEPTH_MODE_NFOV_UNBINNED
            device_config.camera_fps = K4A_FRAMES_PER_SECOND_30
            device_config.wired_sync_mode = K4A_WIRED_SYNC_MODE_STANDALONE
            device_config.synchronized_images_only = True

            device = pykinect.start_device(config=device_config)

            # 获取相机校准参数
            calibration = device.get_calibration(depth_mode=device_config.depth_mode, 
                                                    color_resolution=device_config.color_resolution
                                                )
            transformation = pykinect.Transformation(calibration)

            print("calibration: ",calibration)
            print("transformation: ",transformation.depth_image_to_color_camera)
            while True:
                # results_detect.clear()
                capture = device.update()
                ret = False
                retd = False
                ret, color_frame = capture.get_color_image()
                if depth:
                    # ret, color_frame = capture.get_transformed_color_image()
                    # print("channels: ",color_frame.shape[2])
                    # color_frame = color_frame[:, :, :]

                    # retd, depth_frame = capture.get_depth_image()
                # else:
                #     ret, color_frame = capture.get_color_image()
                #     print("channels1: ",color_frame.shape)
                    retd, depth_image = capture.get_transformed_depth_image()
                #     print("channels2: ",depth_frame.shape)

                if not ret or (depth and not retd):
                    continue

                self.color_frame = color_frame

                # if depth:
                #     # 对齐深度图到RGB图像
                #     transformed_depth_image = transformation.depth_image_to_color_camera(depth_frame)
                #     depth_image = transformed_depth_image
                #     print("size:",depth_image.shape)


                yoloresults = self.pred(depth,target)
                if not yoloresults:
                    print("There is no the thing you want, continue finding")
                    continue

                height, width, channel = color_frame.shape

                resolution = [width,height]
                self.show(resolution,range)
                # if not yoloresults:
                #     print("There is no the thing you want, continue finding")
                #     continue

                if self.judge(pattern,yoloresults,target,width):
                    if pattern=='find' and depth:
                        results_detect.clear()
                        # 结果是目标物品在相机坐标系中的三维坐标，是一个列表，存储了x y z

                        # self.world()利用参数矩阵手动计算
                        point = self.world(camera,depth_image,target,yoloresults)
                        result = {target:point}
                        results_detect.append(result)
                        cv2.imwrite("target.jpg",self.color_frame)
                        
                        # 此处利用 pykinect 内置函数直接计算
                        # for yolor in yoloresults:
                        #     if yolor.name == target:
                        #         x = yolor.x
                        #         y = yolor.y
    
                        #         d = depth_image[int(y),int(x)]
                        #         pixels = k4a_float2_t((y, x))
                        #         if d > 0:  # 确保深度值有效
                        #             # 计算三维坐标
                        #             xyz = calibration.convert_2d_to_3d(pixels, d, K4A_CALIBRATION_TYPE_COLOR, K4A_CALIBRATION_TYPE_COLOR)
                        #             xyz_depth = calibration.convert_2d_to_3d(pixels, d, K4A_CALIBRATION_TYPE_COLOR, K4A_CALIBRATION_TYPE_DEPTH)
                        #             print("xyz: ",xyz.xyz)
                        #             # print("xyz_depth: ",xyz_depth.xyz)
                        #             xyz=xyz_depth
                        #             X, Y, Z = xyz.xyz.x/1000.0, xyz.xyz.y/1000.0, xyz.xyz.z/1000.0
                        #             point = [X, Y, Z]
                        #             result = {target:point}
                        #             results_detect.append(result)
                        #         else:
                        #             print("Invalid depth value, skip!")
                        #         break
                        # if d>0:
                        #     break
                        # else:
                        #     continue
                                
                    elif pattern == 'find':
                        # 结果是目标物品在图像坐标系中的二维坐标，是一个元祖，储存了x y
                        for yolor in yoloresults:
                            if yolor.name == target:
                                x = yolor.x
                                y = yolor.y
                                center = (x,y)
                                result = {target:center}
                        results_detect.append(result)
                    else:
                        # 结果是被检测出的所有物品及其在图像坐标系中的二维坐标，此处暂无深度信息，如有需要可调用world()计算
                        for yolor in yoloresults:
                            name = yolor.name
                            x = yolor.x
                            y = yolor.y
                            center = (x,y)
                            result = {name:center}
                            results_detect.append(result)
                    break

                key_pressed = cv2.waitKey(10)  # 每隔多少毫秒毫秒，获取键盘哪个键被按下
                # print('键盘上被按下的键：', key_pressed)

                if key_pressed in [ord('q'), 27]:  # 按键盘上的q或esc退出.
                    break
            
            device.stop_cameras()
            device.close()

        elif camera=='realsense':
            print("camera is realsense")
            # 1. 配置RealSense相机流
            pipeline = rs.pipeline()
            config = rs.config()
            config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
            config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

            # 启动相机流
            pipeline.start(config)
            # 深度图像向彩色对齐
            depth_to_color = rs.align(rs.stream.color)

            # 获取相机的内参矩阵
            profile = pipeline.get_active_profile()
            intr = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
            fx, fy = intr.fx, intr.fy
            cx, cy = intr.ppx, intr.ppy

            self.K_realsense = np.array([fx, 0, cx, 0, fy, cy, 0, 0, 1]).reshape(3,3)
            print("K_realsense:\n",self.K_realsense)

            while True:
                results_detect.clear()
                # 2. 捕获对齐后的彩色和深度图像
                frames = pipeline.wait_for_frames()
                # color_frame = frames.get_color_frame()
                # depth_frame = frames.get_depth_frame()
                aligned_frames = depth_to_color.process(frames)
                aligned_depth_frame = aligned_frames.get_depth_frame()
                color_frame = aligned_frames.get_color_frame()

                if not color_frame or not aligned_depth_frame:
                    continue

                # 将图像转换为numpy数组
                color_image = np.asanyarray(color_frame.get_data())
                depth_image = np.asanyarray(aligned_depth_frame.get_data())

                self.color_frame = color_image
                height, width, channel = color_image.shape

                yoloresults = self.pred(depth, target)

                resolution = [width,height]
                self.show(resolution,range)

                if self.judge(pattern,yoloresults,target,width):
                    if pattern=='find' and depth:
                        # 结果是目标物品在相机坐标系中的三维坐标，是一个列表，存储了x y z
                        point = self.world(camera,depth_image,target,yoloresults)
                        result = {target:point}
                        if point[2]<=0:
                            continue
                        results_detect.append(result)
                    elif pattern == 'find':
                        # 结果是目标物品在图像坐标系中的二维坐标，是一个元祖，储存了x y
                        for yolor in yoloresults:
                            if yolor.name == target:
                                x = yolor.x
                                y = yolor.y
                                center = (x,y)
                                result = {target:center}
                        results_detect.append(result)
                    else:
                        # 结果是被检测出的所有物品及其在图像坐标系中的二维坐标，此处暂无深度信息，如有需要可调用world()计算
                        for yolor in yoloresults:
                            name = yolor.name
                            x = yolor.x
                            y = yolor.y
                            center = (x,y)
                            result = {name:center}
                            results_detect.append(result)
                    break

                key_pressed = cv2.waitKey(10)  # 每隔多少毫秒毫秒，获取键盘哪个键被按下
                # print('键盘上被按下的键：', key_pressed)

                if key_pressed in [ord('q'), 27]:  # 按键盘上的q或esc退出.
                    break

            # 5. 释放资源
            pipeline.stop()

        elif camera=='astra':
            openni2.initialize()

            dev = openni2.Device.open_any()
            print("openni2:", dev.get_device_info())

            width = 640
            height = 480

            color_stream = dev.create_color_stream()
            color_stream.start()
            if depth:
                depth_stream = dev.create_depth_stream()
                # 彩色和深度图像对齐
                # dev.set_image_registration_mode(True)
                depth_stream.start()

            time.sleep(3)
            while True:
                results_detect.clear()

                cframe = color_stream.read_frame()
                if depth:
                    dframe = depth_stream.read_frame()

                    dframe_data = np.array(dframe.get_buffer_as_triplet()).reshape([480, 640, 2])
                    dpt1 = np.asarray(dframe_data[:, :, 0], dtype='float32')
                    dpt2 = np.asarray(dframe_data[:, :, 1], dtype='float32')
                    dpt2 *= 255 # 高八位
                    depth_image = dpt1 + dpt2
                    cv2.imshow('depth', depth_image)
                
                cframe_data = np.array(cframe.get_buffer_as_triplet()).reshape([height, width, 3])
                self.color_frame = cv2.cvtColor(cframe_data, cv2.COLOR_RGB2BGR)

                yoloresults = self.pred()

                resolution = [width,height]
                self.show(resolution,range)

                if self.judge(pattern,yoloresults,target,width):
                    if pattern=='find' and depth:
                        # 结果是目标物品在相机坐标系中的三维坐标，是一个列表，存储了x y z
                        point = self.world(camera,depth_image,target,yoloresults)
                        result = {target:point}
                        results_detect.append(result)
                    elif pattern == 'find':
                        # 结果是目标物品在图像坐标系中的二维坐标，是一个元祖，储存了x y
                        for yolor in yoloresults:
                            if yolor.name == target:
                                x = yolor.x
                                y = yolor.y
                                center = (x,y)
                                result = {target:center}
                        results_detect.append(result)
                    else:
                        # 结果是被检测出的所有物品及其在图像坐标系中的二维坐标，此处暂无深度信息，如有需要可调用world()计算
                        for yolor in yoloresults:
                            name = yolor.name
                            x = yolor.x
                            y = yolor.y
                            center = (x,y)
                            result = {target:center}
                            results_detect.append(result)
                    break

                key_pressed = cv2.waitKey(10)  # 每隔多少毫秒毫秒，获取键盘哪个键被按下
                # print('键盘上被按下的键：', key_pressed)

                if key_pressed in [ord('q'), 27]:  # 按键盘上的q或esc退出.
                    break

            color_stream.stop()
            if depth:
                depth_stream.stop()
            dev.close()

        else:
            # 使用电脑摄像头
            cap = cv2.VideoCapture(0)  # 0 is the default camera index
            # cap.open(0)
            # 获取摄像头的默认分辨率
            width = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
            height = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
            print(f"width:{width}, height:{height}")
            while cap.isOpened():
                results_detect.clear()
                ret, frame = cap.read()
                if not ret:
                    break
                self.color_frame = frame

                yoloresults = self.pred()

                resolution = [width,height]
                self.show(resolution,range)

                if self.judge(pattern,yoloresults,target,width):
                    if pattern == 'find':
                        # 结果是目标物品在图像坐标系中的二维坐标
                        for yolor in yoloresults:
                            if yolor.name == target:
                                x = yolor.x
                                y = yolor.y
                                center = (x,y)
                                result = {target:center}
                        results_detect.append(result)
                        # center = centers[names.index(target)]
                        # result = {target:center}
                        # results_detect.append(result)
                    else:
                        # 结果是被检测出的所有物品及其在图像坐标系中的二维坐标，此处暂无深度信息，如有需要可调用world()计算
                        for yolor in yoloresults:
                            name = yolor.name
                            x = yolor.x
                            y = yolor.y
                            center = (x,y)
                            result = {target:center}
                            results_detect.append(result)
                    break

                key_pressed = cv2.waitKey(10)  # 每隔多少毫秒毫秒，获取键盘哪个键被按下
                # print('键盘上被按下的键：', key_pressed)

                if key_pressed in [ord('q'), 27]:  # 按键盘上的q或esc退出.
                    break

            cap.release()

        cv2.destroyAllWindows()

        return results_detect

        
    def pred(self, depth_, target_):
        """
        推理，返回YoloResult类型结果
        """

        results = self.model(self.color_frame)
        self.color_frame = results[0].plot()
        model_names = results[0].names
        # print("results:",results[0][0])
        # Extract and display bounding box information

        # xyxy = []
        # names = []
        # confs = []
        yoloresults = []
        # height,width,channels = img.shape
        # print(f"Width: {width}, Height: {height}")

        for result in results[0]:
            box = result.boxes
            x1, y1, x2, y2 = map(int, box.xyxy[0])  # Bounding box coordinates
            conf = box.conf[0]  # Confidence score
            cls = box.cls[0]  # Class label index
            print(f"Class: {results[0].names[int(cls)]}, Confidence: {conf:.2f}, Box: [{x1}, {y1}, {x2}, {y2}]")

            center_x = (x1+x2)/2
            center_y = (y1+y2)/2
            # center = (center_x,center_y)
            yoloresult = YoloResult(model_names[int(cls)],[x1,y1,x2,y2],center_x,center_y,conf)

            if depth_:
                if yoloresult.name == target_:
                    yoloresults.append(yoloresult)
            else:
                yoloresults.append(yoloresult)

            # xyxy.append([x1,y1,x2,y2])
            # names.append(model_names[int(cls)])
            # confs.append(conf)

        # return xyxy,names,results,confs

        return yoloresults


    def center(self,boxes):
        """
            根据瞄框的左上角和右下角坐标计算中心点
        """
        centers = []
        for box in boxes:
            center_x = (box[0]+box[2])/2
            center_y = (box[1]+box[3])/2
            centers.append((center_x,center_y))

        return centers
    
    def show(self,resolution,range):
        width = resolution[0]
        height = resolution[1]
        cv2.line(self.color_frame, (int(width * 0.5 * (1 - range)), 0), (int(width * 0.5 * (1 - range)), int(height)),
                    (0, 255, 0), 2, 4)
        cv2.line(self.color_frame, (int(width * 0.5 * (1 + range)), 0), (int(width * 0.5 * (1 + range)), int(height)),
                    (0, 255, 0), 2, 4)
        
        # for center in centers:
        #     cv2.circle(self.color_frame, (int(center[0]),int(center[1])), 1, (0, 0, 255), 8)
        # 展示处理后的三通道图像
        cv2.imshow('yolo', self.color_frame)

    def judge(self,pattern,yolors,target=None,resolution=640,range=0.8):
        flag = False
        if pattern == "realtime":
            # 实时检测
            flag = cv2.waitKey(1) & 0xFF == ord('q')
        elif pattern == 'find':
            # 寻找目标
            for yolor in yolors:
                if target == yolor.name:
                    x = yolor.x
                    flag = cv2.waitKey(1) and self.judge_range(x, resolution, range) and (yolor.conf>0.5)
                    if flag:
                        break
        else:
            if len(yolors)>0:
                # 如果感兴趣区域内没有可被检测的物品则会一直检测，直到在感兴趣区域中出现了可被检测的物品
                for yolor in yolors:
                    x = yolor[0].x
                    flag = cv2.waitKey(1) and self.judge_range(x, resolution, range)
                    if flag:
                        break
        
        return flag

    
    def judge_range(self, x, resolution=640, range=0.8):
        """
            判断目标点是否在感兴趣范围内
        """
        left = resolution * 0.5 * (1 - range)
        right = resolution * 0.5 * (1 + range)
        return left <= x <= right
    
    def world(self,camera,depth_image,target,yolors):
        """
            计算目标在相机坐标系下的三维坐标
        """
        if camera=='k4a' or camera=='kinect':
            K = self.K_kinect
        elif camera=='realsense':
            K = self.K_realsense
        elif camera=='astra':
            K = self.K_astra
        else:
            return ValueError("camera input error, input again please")
        
        # index = names.index(target)
        # point = centers[index]
        # x = point[0]
        # y = point[1]
        x = -1
        y = -1
        # for yolor in yolors:
        #     if target == yolor.name:
        #         x = yolor.x
        #         y = yolor.y
        #         break
        x = yolors[0].x
        y = yolors[0].y
        print("x,y:",x,y)
        # pixel_coords = np.array([])
        z = depth_image[int(y),int(x)] * 0.001
        print("z:",z)
        
        point_image = np.array([x,y,1])  # 图像坐标系
        # point = np.dot(K, point_image) * z*10e-4 # 相机坐标系
        point = z * np.linalg.inv(K).dot(point_image)

        return point


if __name__ == "__main__":
    detector = Detector()
    results = detector.detect(camera='k4a',pattern='find',target='bottle',depth=True,range=0.8)
    print("len results:", len(results))
    if len(results)>0:
        (name, point), = results[0].items()
        print(f"Class:{name}, Point:{point}")