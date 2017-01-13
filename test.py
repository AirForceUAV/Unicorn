def movement3(channel, percent=0):
    # -100<=percent<=100
    sign = 0
    if percent < 0:
        sign = -1
    elif percent > 0:
        sign = 1
    rate = percent / 100.0
    index = 2 + channel[5] * sign
    section = abs(channel[2] - channel[index])
    variation = int(rate * channel[5] * section)
    result = channel[2] + variation
    print index, variation, result
    return result


def movement4(channel, percent=0):
    # 0<=percent<=100
    section = channel[3] - channel[1]
    rate = percent / 100.0
    if channel[5] < 0:
        rate = 1 - rate
    variation = int(rate * section)
    result = channel[1] + variation
    print result
    return result

a = [3, 159, 827, 1503, 1344, 1, 3]
b = [3, 159, 827, 1503, 1344, -1, 3]

# movement3(a, 100)
# movement3(a, 0)
# movement3(a, -100)

# movement3(b, 100)
# movement3(b, 0)
# movement3(b, -100)

movement4(a, 0)
movement4(a, 50)
movement4(a, 100)

movement4(b, 0)
movement4(b, 50)
movement4(b, 100)
