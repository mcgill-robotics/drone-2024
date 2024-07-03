import cv2
import numpy as np
from ultralytics import YOLO
import sys


def generate_mask(img,
                  hue_ranges,
                  saturation_range,
                  value_range,
                  additional_mask=None):
    """
        img is an HSV image
        hue_ranges is an iterable of tuples with lower and upper degrees for
            hue.(one smaller than the other)
        saturation_range is a tuple of lower and upper saturation percentages.
        value_range is a tuple of lower and upper value percentages.
    """

    def hue_to_cv(hue_deg):
        return hue_deg * 180 / 360

    def sat_val_to_cv(sat_val_perc):
        return sat_val_perc * 255 / 100

    mask = None
    lower_sat, upper_sat = saturation_range
    lower_sat, upper_sat = sat_val_to_cv(lower_sat), sat_val_to_cv(upper_sat)
    lower_val, upper_val = value_range
    lower_val, upper_val = sat_val_to_cv(lower_val), sat_val_to_cv(upper_val)
    for lower_hue, upper_hue in hue_ranges:
        lower_hue, upper_hue = hue_to_cv(lower_hue), hue_to_cv(upper_hue)
        lower = np.array([lower_hue, lower_sat, lower_val])
        upper = np.array([upper_hue, upper_sat, upper_val])
        new_mask = cv2.inRange(img, lower, upper)
        if mask is None:
            mask = new_mask
        else:
            mask = cv2.bitwise_or(mask, new_mask)
    if (additional_mask is not None):
        mask = cv2.bitwise_and(mask, additional_mask)
    return mask


def get_color_dict(hsv_img, additional_mask=None):
    colors = {
        'red': 0.,
        'green': 0.,
        'blue': 0.,
        'white': 0.,
        'black': 0.,
        'purple': 0.,
        'brown': 0.,
        'orange': 0.
    }

    red_mask = generate_mask(hsv_img, [(0, 12), (332, 360)], (30, 100),
                             (30, 100), additional_mask)

    green_mask = generate_mask(hsv_img, [(70, 150)], (30, 100), (30, 100),
                               additional_mask)

    blue_mask = generate_mask(hsv_img, [(150, 248)], (30, 100), (30, 100),
                              additional_mask)

    white_mask = generate_mask(hsv_img, [(0, 360)], (0, 30), (40, 100),
                               additional_mask)

    black_mask = generate_mask(hsv_img, [(0, 360)], (0, 100), (0, 30),
                               additional_mask)

    purple_mask = generate_mask(hsv_img, [(248, 332)], (30, 100), (30, 100),
                                additional_mask)

    brown_mask = generate_mask(hsv_img, [(0, 70), (332, 360)], (30, 100),
                               (30, 70), additional_mask)

    orange_mask = generate_mask(hsv_img, [(12, 70)], (30, 100), (70, 100),
                                additional_mask)

    colors['red'] = np.sum(red_mask)
    colors['green'] = np.sum(green_mask)
    colors['blue'] = np.sum(blue_mask)
    colors['white'] = np.sum(white_mask)
    colors['black'] = np.sum(black_mask)
    colors['purple'] = np.sum(purple_mask)
    colors['brown'] = np.sum(brown_mask)
    colors['orange'] = np.sum(orange_mask)

    return colors


def read_color_on_shape(shape_crop, letter_x1, letter_y1, letter_x2,
                        letter_y2):
    # process object
    hsv_img = cv2.cvtColor(shape_crop, cv2.COLOR_BGR2HSV)
    grey_img = cv2.cvtColor(shape_crop, cv2.COLOR_BGR2GRAY)
    contours, _ = cv2.findContours(grey_img.copy(), cv2.RETR_EXTERNAL,
                                   cv2.CHAIN_APPROX_SIMPLE)

    contour_mask = np.zeros(hsv_img.shape[:2], np.uint8)
    cv2.drawContours(contour_mask, contours, -1, (255), cv2.FILLED)
    cv2.imshow("Contour", contour_mask)
    cv2.waitKey(0)

    colors_shape = get_color_dict(hsv_img, contour_mask)
    keymax = sorted(zip(colors_shape.values(), colors_shape.keys()),
                    key=lambda x: x[0])
    shape_color = keymax[-1][1]

    letter_hsv = hsv_img[int(letter_y1):int(letter_y2),
                         int(letter_x1):int(letter_x2)]

    color_letter = get_color_dict(letter_hsv)
    keymax = sorted(zip(color_letter.values(), color_letter.keys()),
                    key=lambda x: x[0])
    first_letter_color, second_letter_color = keymax[-1][1], keymax[-2][1]
    letter_color = None
    if (shape_color == first_letter_color):
        letter_color = second_letter_color
    else:
        letter_color = first_letter_color
    return shape_color, letter_color


def main(path):
    img = cv2.imread(path, cv2.IMREAD_COLOR)
    letter_detection_model = YOLO('runs/detect/letters_yolo/weights/best.pt')
    letter_detections = letter_detection_model(img)[0].boxes.data.tolist()[0]
    x1_model, y1_model, x2_model, y2_model, score, id = letter_detections
    cv2.imshow("image", img)
    cv2.waitKey(0)
    print(read_color_on_shape(img, x1_model, y1_model, x2_model, y2_model))


if __name__ == "__main__":
    path = sys.argv[1]
    main(path)
