import os
import argparse
from ultralytics import YOLO
model = YOLO('./best_A2024_noAC_int8_openvino_model/')
CONF_THR = 0.75
def process(res):
    conf = res[0].boxes.conf.numpy()
    bbox = res[0].boxes.xywhn
    if conf > CONF_THR and len(conf) > 0:
        return bbox.numpy()
    else:
        return []


def inference(root):
    result = {}
    for filename in os.listdir(root):
        if filename[-3:] != 'txt' and ('.ipynb_checkpoints' not in filename ) :
            res = model.predict(root + filename)
            bbox = process(res)
            if len(bbox) != 0:
                x, y = bbox[0][:2]
                result[filename] = [x, y]
    return result

def z_to_w(z):
    w = z * 18 / 16
    return w

def z_to_h(z):
    h = z * 12 / 16
    return h

def calculate_coordinates(x_gps, y_gps, z_gps, x_obj, y_obj):
    w_real = z_to_w(z_gps)
    h_real = z_to_h(z_gps)
    x_obj_real = x_gps - w_real * (x_obj - 0.5)
    y_obj_real = y_gps + h_real * (y_obj - 0.5)
    return True, x_obj_real, y_obj_real

# Firstly parse coordinates, then NN output
def parse_coordinates(path):
    global img_info_list
    with open(path, 'r') as coordinates_file:
        coordinates = coordinates_file.readlines()
    for img_info_line in coordinates:
        name, x_gps, y_gps, z_gps = img_info_line.split(' ')
        x_gps, y_gps, z_gps = list(map(float, [x_gps, y_gps, z_gps]))
        img_info = {
            'name': name,
            'gps': [x_gps, y_gps, z_gps],
        }
        img_info_list += [img_info]

# Firstly parse coordinates, then NN output
def parse_nn_output(nn_output):
    global img_info_list
    for info_id in range(len(img_info_list)):
      if img_info_list[info_id]["name"] in nn_output:
        x_obj, y_obj = nn_output[f'{img_info_list[info_id]["name"]}']
        x_gps, y_gps, z_gps = img_info_list[info_id]['gps']
        x_obj_real, y_obj_real = calculate_coordinates(x_gps, y_gps, z_gps, x_obj, y_obj)
        img_info_list[info_id]['obj'] = [x_obj_real, y_obj_real]
      else:
        img_info_list[info_id]['obj'] = None

def show_results():
    global img_info_list
    for img_info in img_info_list:
        try:
            print(f'Name:\t{img_info["name"]}')
        except:
            print(f'Name:\tNone')
        try:
            print(f'GPS:\t{img_info["gps"]}')
        except:
            print(f'GPS:\tNone')
        try:
            print(f'Object:\t{img_info["obj"]}')
        except:
            print(f'Object:\tNone')
        print('')

img_info_list = []

def run(root):
    nn_output = inference(root)
    parse_coordinates(root + 'coordinates.txt')
    parse_nn_output(nn_output)
    show_results()

if __name__=="__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--path_images', default = './DataExample/')
    args = parser.parse_args()
    run(args.path_images)