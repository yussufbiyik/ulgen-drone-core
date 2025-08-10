import math

def low_pass_filter(value, prev_value, alpha):
    """
    Herhangi bir değere low-pass filtre uygular.
    Bu, değerlerin daha pürüzsüz ve daha az gürültülü olmasını sağlar.
    """
    return alpha * value + (1 - alpha) * prev_value