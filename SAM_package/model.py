from .singleton import SingletonMeta
from segment_anything import SamPredictor, sam_model_registry


class SAM_client(metaclass=SingletonMeta):
    def __init__(self):
        sam_checkpoint = "/home/ubuntu/sam_vit_h_4b8939.pth"
        model_type = "vit_h"
        sam = sam_model_registry[model_type](checkpoint=sam_checkpoint)
        self.predictor = SamPredictor(sam)
        print('predictor initialized')

    def set_image(self, image):
        return self.predictor.set_image(image)

    def predict(self, input_point, input_label, multimask_bool):
        masks, scores, logits = self.predictor.predict(
            point_coords=input_point,
            point_labels=input_label,
            multimask_output=multimask_bool
        )
        return masks, scores, logits