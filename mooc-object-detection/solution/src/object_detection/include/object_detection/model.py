import ctypes
import os


def run(input, exception_on_failure=False):
    print(input)
    try:
        import subprocess
        program_output = subprocess.check_output(f"{input}", shell=True, universal_newlines=True,
                                                 stderr=subprocess.STDOUT)
    except Exception as e:
        if exception_on_failure:
            print(e.output)
            raise e
        program_output = e.output
    print(program_output)
    return program_output.strip()

class Wrapper():
    def __init__(self, _):
        dt_token = "dt1-3nT8KSoxVh4MdLnE1Bq2mTkhRpbR35G8mmbjExH5deTkpsN-43dzqWFnWd8KBa1yev1g3UKnzVxZkkTbffvW31zEoh35fcbiTrhMQoFvGEH9ztHXBc" # TODO you must add your token here!
        model_name = "yolov5" # TODO you must add your model's name here!

        cache_path = "/code/solution/nn_models"
        from dt_device_utils import DeviceHardwareBrand, get_device_hardware_brand
        if get_device_hardware_brand() == DeviceHardwareBrand.JETSON_NANO:
            cache_path = "/data/nn_models"
        if not os.path.exists(cache_path):
            os.makedirs(cache_path)

        from dt_mooc.cloud import Storage
        storage = Storage(dt_token)
        storage.cache_directory = cache_path    # todo this is dirty fix upstram in lib

        file_already_existed = storage.is_hash_found_locally(model_name, cache_path)

        storage.download_files(model_name, cache_path)

        weight_file_path = f"{storage.cache_directory}/{model_name}"

        if get_device_hardware_brand() == DeviceHardwareBrand.JETSON_NANO and not file_already_existed:
            # https://github.com/duckietown/tensorrtx/tree/dt-yolov5/yolov5
            run("git clone https://github.com/duckietown/tensorrtx.git -b dt-obj-det")
            run(f"cp {weight_file_path}.wts ./tensorrtx/yolov5.wts")
            run(f"cd tensorrtx && ls && chmod 777 ./do_convert.sh && ./do_convert.sh", exception_on_failure=True)
            run(f"mv tensorrtx/build/yolov5.engine {weight_file_path}.engine")
            run(f"mv tensorrtx/build/libmyplugins.so {weight_file_path}.so")
            run("rm -rf tensorrtx")

        if get_device_hardware_brand() == DeviceHardwareBrand.JETSON_NANO:
            self.model = TRTModel(weight_file_path)
        else:
            self.model = AMD64Model(weight_file_path)


    def predict(self, image):
        # TODO Make your model predict here!

        # TODO you should call self.model's infer function, and filter the predictions for the information you want.
        # TODO For example, you probably only care about duckies, and maybe even only about duckies whose bounding boxes
        # TODO are above a certain size, so as to stop only when a duckie is close to your duckiebot.

        return self.model.infer(image)

class Model():
    def __init__(self):
        pass
    def infer(self, image):
        raise NotImplementedError()

class AMD64Model():
    def __init__(self, weight_file_path):
        super().__init__()

        import torch
        self.model = torch.hub.load('ultralytics/yolov5', 'custom', path=f'{weight_file_path}.pt')
        if torch.cuda.is_available():
            self.model = self.model.cuda()
        else:
            self.model = self.model.cpu()

    def infer(self, image):
        det = self.model(image, size=416)

        xyxy = det.xyxy[0]  # grabs det of first image (aka the only image we sent to the net)

        print(xyxy)

        if xyxy.shape[0] > 0:
            conf = xyxy[:,-2]
            clas = xyxy[:,-1]
            xyxy = xyxy[:,:-2]

            return xyxy, clas, conf
        return [], [], []

class TRTModel(Model):
    def __init__(self, weight_file_path):
        super().__init__()
        ctypes.CDLL(weight_file_path+".so")
        from tensorrt_model import YoLov5TRT
        self.model = YoLov5TRT(weight_file_path+".engine")
    def infer(self, image):
        # todo ensure this is in boxes, classes, scores format
        return self.model.infer_for_robot([image])