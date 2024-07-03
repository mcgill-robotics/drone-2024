import string
import cv2
import numpy as np


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


def draw_text(img,
              text,
              font=cv2.FONT_HERSHEY_PLAIN,
              pos=(0, 0),
              font_scale=3,
              font_thickness=2,
              text_color=(0, 255, 0),
              text_color_bg=(0, 0, 0)):

    x, y = pos
    x, y = int(x), int(y)
    pos = (x, y)
    text_size, _ = cv2.getTextSize(text, font, font_scale, font_thickness)
    text_w, text_h = text_size
    cv2.rectangle(img, pos, (int(x + text_w), int(y + text_h)), text_color_bg,
                  -1)
    cv2.putText(img, text, (int(x), int(y + text_h + font_scale - 1)), font,
                font_scale, text_color, font_thickness)

    return text_size


def find_alphanum(detection, detections_object, frame_of_interest, frame_copy,
                  x1, x2, y1, y2):
    W, H, _ = frame_copy.shape
    x1_model, y1_model, x2_model, y2_model, score, id = detection
    text = detections_object[0].names[id]
    # read letters or numbers and colors
    shape_color, letter_color = read_color_on_shape(frame_of_interest,
                                                    x1_model, y1_model,
                                                    x2_model, y2_model)
    print(
        f"FOUND TEXT {text}, shape color {shape_color}, letter color {letter_color}"
    )
    font = cv2.FONT_HERSHEY_SIMPLEX
    text = f"{text}, {score*100:0.2f}%, {letter_color, shape_color}"
    draw_text(frame_copy,
              text,
              pos=(int(x1 - 22), int(y1 - 22)),
              font_scale=1,
              text_color=(0, 255, 0))
    cv2.rectangle(frame_copy,
                  (max(0, int(x1_model + x1)), max(0, int(y1_model + y1))),
                  (min(int(x2_model + x1), W), min(int(y2_model + y1), H)),
                  (0, 255, 0), 2)
    return shape_color, letter_color, text


def rect_text_img(img, text, x1, y1, x2, y2, color=(255, 255, 255)):
    draw_text(img, text, pos=(x1, y1 - 30), text_color=color)
    cv2.rectangle(img, (int(x1), int(y1)), (int(x2), int(y2)), color, 2)
