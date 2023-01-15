def get_model_by_type():

    input_shape = (120, 160, 3)
    roi_crop = (0,0)

    from .keras import KerasLinear
    kl = KerasLinear(input_shape = input_shape, roi_crop = roi_crop)
    return kl