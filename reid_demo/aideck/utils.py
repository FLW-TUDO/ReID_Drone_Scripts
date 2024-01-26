import numpy as np
import cv2
import os

def colorBalance(img):
    '''
    Function to color balance wrt a white object
    '''
    mean = [0,0,0]
    factor = [1,1,1]
    img_out = img.copy()
    for c in range(3):
        mean[c] = np.mean(img[:,:,c].flatten())
        factor[c] = 255.0/mean[c]
        img_out[:,:,c] = np.clip(img_out[:,:,c]*factor[c],0,255)

    print("Factors: {}".format(factor))
    return img_out, factor

def colorCorrectBayer(img_, factors=[1,1,1]):
    '''
    Color correction for the RGB Camera. It has a sensor with a Bayer pattern, which has
    more green cells than blue and red, so if the image is not treated, it will have  a
    green-ish look.
    '''
    # TODO: Apply an actual color correction without luminosity loss. -> histogram level
    # This is just an approximation
    img = img_.copy()
    for i in range(3):
        img[:,:,i] = np.clip(img[:,:,i]*factors[i],0,255)
    return img

def rx_bytes(size, client_socket):
    '''
    Read N bytes from the socket
    '''
    data = bytearray()
    while len(data) < size:
        data.extend(client_socket.recv(size-len(data)))
    return data

def draw_scan_area(resized):
    color = [40, 220, 50]
    left_dist=0.1
    right_dist=0.9
    top_dist=0.1
    bot_dist=0.85
    length = 0.15
    thickn=4
    pt_tl=(int(resized.shape[1]*left_dist), int(resized.shape[0]*top_dist))
    pt_tl_r=(int(resized.shape[1]*(0.1+length)), int(resized.shape[0]*top_dist))
    pt_tl_b=(int(resized.shape[1]*0.1), int(resized.shape[0]*(top_dist+length)))
    pt_tr=(int(resized.shape[1]*right_dist), int(resized.shape[0]*top_dist))
    pt_tr_l=(int(resized.shape[1]*(right_dist-length)), int(resized.shape[0]*top_dist))
    pt_tr_b=(int(resized.shape[1]*right_dist), int(resized.shape[0]*(top_dist+length)))
    pt_bl=(int(resized.shape[1]*left_dist), int(resized.shape[0]*bot_dist))
    pt_bl_r=(int(resized.shape[1]*(left_dist+length)), int(resized.shape[0]*bot_dist))
    pt_bl_t=(int(resized.shape[1]*left_dist), int(resized.shape[0]*(bot_dist-length)))
    pt_br=(int(resized.shape[1]*right_dist), int(resized.shape[0]*bot_dist))
    pt_br_l_=(int(resized.shape[1]*(right_dist-length)), int(resized.shape[0]*bot_dist))
    pt_br_t=(int(resized.shape[1]*right_dist), int(resized.shape[0]*(bot_dist-length)))
    cv2.line(resized, pt_tl, pt_tl_r, color, thickn)
    cv2.line(resized, pt_tl, pt_tl_b, color, thickn)
    cv2.line(resized, pt_tr, pt_tr_l, color, thickn)
    cv2.line(resized, pt_tr, pt_tr_b, color, thickn)
    cv2.line(resized, pt_bl, pt_bl_r, color, thickn)
    cv2.line(resized, pt_bl, pt_bl_t, color, thickn)
    cv2.line(resized, pt_br, pt_br_l_, color, thickn)
    cv2.line(resized, pt_br, pt_br_t, color, thickn)
    return resized

def draw_crosshair(image, box, length=10, color=[40, 220, 50]):
    """
        args:
            box : tuple(int, int, int, int) -> (x to left, y top left, width, height)
    """
    thickn=2
    center = (box[0] + box[2]/2, box[1] + box[3]/2)
    vertical_line_l = (int(center[0]), int(center[1]  - length/2))
    vertical_line_r = (int(center[0]), int(center[1]  + length/2))

    horizontal_line_l = (int(center[0] - length/2), int(center[1]))
    horizontal_line_r = (int(center[0] + length/2), int(center[1]))

    cv2.line(image, horizontal_line_l, horizontal_line_r, color, thickn)
    cv2.line(image, vertical_line_l, vertical_line_r, color, thickn)

    return image

def detect_box(resized, model, conf, nms, color=(0, 225, 0)):
    class_names = ['palletblock']
    classes, scores, boxes = model.detect(resized, conf, nms)
    box_results = []
    if len(boxes) > 0:
        for (classid, score, box) in zip(classes, scores, boxes):
            resized = draw_filled_rect(resized, box, str(round(100*score)), rect_color=color)
            resized = draw_crosshair(resized, box)
            box_results.append([*box, score*100])
        # draw chosen bb
        box = choose_most_left_bb(box_results)
        resized = draw_crosshair(resized, box, color=[168, 50, 70])
        return box_results, resized
    else:
        return None, resized
    
def draw_filled_rect(img_original, box, label, rect_color=(0, 225, 0), text_color=[10, 220, 10], alpha=0.05):
    img_rect_filled = img_original.copy()
    # cv2.rectangle(img_rect_filled, box , color=rect_color, thickness=-1)
    img_processed=cv2.addWeighted(img_rect_filled, alpha, img_original, 1 - alpha, 0)
    img_processed=cv2.rectangle(img_processed, box , color=rect_color, thickness=3)
    cv2.putText(img_processed, label + "%", (int(box[0] + box[2]),int(box[1])), cv2.FONT_HERSHEY_SIMPLEX, 0.7, text_color, 3)
    return img_processed

def draw_detection_marker(img_original, box, label):
    color = [0, 255, 255]
    img_rect_filled = img_original.copy()
    cv2.putText(img_rect_filled, label, (int(box[0] + box[2] / 2), int(box[1] + box[3] / 2)), cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 1)
    return img_rect_filled

def resize_frame(frame, resize_factor=1.0):
    width = int(frame.shape[1] * resize_factor)
    height = int(frame.shape[0] * resize_factor)
    dim = (width, height)
    img_original=cv2.resize(frame, dim, interpolation = cv2.INTER_CUBIC)
    return img_original


def choose_most_left_bb(bounding_boxes):
    best_bb, best_bb_score = None, 1000
    for box in bounding_boxes:
        score = box[0]
        if score < best_bb_score:
            best_bb = box[0:-1]
            best_bb_score = score

    return best_bb

def choose_middle_bb(pallet_offsets):
    if pallet_offsets is None:
        return None
    
    offsets = [offset_x for offset_x, _, _, _, _ in pallet_offsets]
    offsets = sorted(offsets)

    return offsets[len(offsets) // 2]

def choose_closest_bb(pallet_offsets):
    if pallet_offsets is None:
        return None

    best_bb, best_bb_score = None, 1000
    for offset_x, offset_y, area, center_x, center_y in pallet_offsets:
        score = abs(offset_x)
        if score < best_bb_score:
            best_bb = (offset_x, offset_y, area, center_x, center_y)
            best_bb_score = score

    return best_bb

def safe_create_folder(name):
    if not os.path.exists(name):
        os.mkdir(name)
    return name

def choose_most_left_bb(bounding_boxes):
    best_bb, best_bb_score = None, 1000
    for box in bounding_boxes:
        score = box[0]
        if score < best_bb_score:
            best_bb = box[0:-1]
            best_bb_score = score

    return best_bb

def calculate_pallet_offsets(pallet_bb):
        """
            # lowerright, upperleft
            pallet_bb : [x,y,x,y]
        """
        upperleft = [pallet_bb[0], pallet_bb[1]]
        lowerright = [pallet_bb[0] + pallet_bb[2], pallet_bb[1] + pallet_bb[3]]

        width = abs(upperleft[0] - lowerright[0])
        height = abs(upperleft[1] - lowerright[1])

        area = width * height

        center_x = upperleft[0] + width//2
        center_y = upperleft[1] + height//2

        offset_x = 324/2 - center_x
        offset_y = 244/2 - center_y

        return offset_x, offset_y, area, center_x, center_y