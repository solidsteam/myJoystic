# print(buma_value_part)


def buma_value(hex_buma, length, print_=False):
    """

    :param hex_buma: hex str or hex num of a buma
    :param length: number of bits in binary
    :return: represented value by this buma
    """
    pow_n = length - 1
    if type(hex_buma) == 'str':
        pass
    else:
        hex_buma = hex(hex_buma)
    py_int_buma = int(hex_buma, 16)
    if py_int_buma > 2 ** pow_n:
        buma_value_part = py_int_buma - 2 ** pow_n
        fanma_value_part = buma_value_part - 1
        yuanma_value_part = 2 ** pow_n - 1 - fanma_value_part
        value = -yuanma_value_part
    else:
        value = py_int_buma
    if print_:
        print(value)
    return value


#
#
if __name__ == '__main__':

    buma_value('0x85', 8)

