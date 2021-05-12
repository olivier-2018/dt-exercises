import ctypes


def run(input, exception_on_failure=False):
    try:
        import subprocess
        program_output = subprocess.check_output(f"{input}", shell=True, universal_newlines=True,
                                                 stderr=subprocess.STDOUT)
    except Exception as e:
        if exception_on_failure:
            print(e.output)
            raise e
        program_output = e.output
    return program_output.strip()

class Wrapper():
    def __init__(self, _):
        dt_token = "dt1-3nT8KSoxVh4MdLnE1Bq2mTkhRpbR35G8mmbjExH5deTkpsN-43dzqWFnWd8KBa1yev1g3UKnzVxZkkTbffvW31zEoh35fcbiTrhMQoFvGEH9ztHXBc" # TODO you must add your token here!
        model_name = "yolov5" # TODO you must add your model's name here!

        from dt_mooc.cloud import Storage
        storage = Storage(dt_token)

        file_already_existed = storage.is_hash_found_locally(model_name)

        storage.download_files(model_name) # TODO you must change this to the filename you used for your model!

        weight_file_path = f"{storage.cache_directory}/{model_name}"

        from dt_device_utils import DeviceHardwareBrand, get_device_hardware_brand
        if get_device_hardware_brand() == DeviceHardwareBrand.JETSON_NANO and not file_already_existed:
            # https://github.com/duckietown/tensorrtx/tree/dt-yolov5/yolov5
            run("git clone https://github.com/duckietown/tensorrtx.git -d dt-yolov5")
            run(f"mv {weight_file_path}.wts tensorrtx/yolov5.wts")
            run(f"cd tensorrtx && ./do_convert.sh", exception_on_failure=True)
            run(f"mv tensorrtx/build/yolov5.engine {weight_file_path}.engine")
            run(f"mv tensorrtx/build/libmyplugins.so {weight_file_path}.so")

        if get_device_hardware_brand() == DeviceHardwareBrand.JETSON_NANO:
            self.model = TRTModel(weight_file_path)
        else:
            self.model = AMD64Model(weight_file_path)

    def predict(self, image):
        # TODO Make your model predict here!

        # TODO you should call self.model's infer function, and filter the predictions for the information you want.
        # TODO For example, you probably only care about duckies, and maybe even only about duckies whose bounding boxes
        # TODO are above a certain size, so as to stop only when a duckie is close to your duckiebot.

        return boxes

class Model():
    def __init__(self):
        pass
    def infer(self, image):
        raise NotImplementedError()

class AMD64Model(Model):
    def __init__(self, weight_file_path):
        super().__init__()

        import torch
        self.model = torch.hub.load('ultralytics/yolov5', 'custom', path=f'{weight_file_path}.pt')
    def infer(self, image):
        return self.model(image)

class TRTModel(Model):
    def __init__(self, weight_file_path):
        super().__init__()
        ctypes.CDLL(weight_file_path+".so")
        from tensorrt_model import YoLov5TRT
        self.model = YoLov5TRT(weight_file_path+".engine")
    def infer(self, image):
        return self.model.infer_for_robot([image])