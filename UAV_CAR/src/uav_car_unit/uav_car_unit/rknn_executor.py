from rknnlite.api import RKNNLite


class RKNN_model_container():
    def __init__(self, model_path) -> None:
        rknn_lite = RKNNLite()

        # Direct Load RKNN Model
        rknn_lite.load_rknn(model_path)

        print('--> Init runtime environment')
        # if target==None:
        #     ret = rknn_lite.init_runtime()
        # else:
        ret = rknn_lite.init_runtime(core_mask=RKNNLite.NPU_CORE_0_1_2)
        if ret != 0:
            print('Init runtime environment failed')
            exit(ret)
        print('done')
        
        self.rknn_lite = rknn_lite

    # def __del__(self):
    #     self.release()

    def run(self, inputs):
        if self.rknn_lite is None:
            print("ERROR: rknn has been released")
            return []

        if isinstance(inputs, list) or isinstance(inputs, tuple):
            pass
        else:
            inputs = [inputs]

        result = self.rknn_lite.inference(inputs=inputs,data_format=['nhwc'])
    
        return result

    def release(self):
        self.rknn_lite.release()
        self.rknn_lite = None