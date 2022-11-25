import os
import shutil
import re

image_path = "/home/issei/images/"
save_path = "/home/issei/sample_images/"
split_val = 2

def atoi(text):
    return int(text) if text.isdigit() else text

def natural_keys(text):
    return [ atoi(c) for c in re.split(r'(\d+)', text) ]


raw_files = os.listdir(image_path)
sorted_files = sorted(raw_files, key=natural_keys)

for i in range(len(sorted_files)):
    filename_num = re.sub(r"\D", "", sorted_files[i])
    if int(filename_num) % split_val == 0:
        # sample_imagesに画像ファイルをコピー
        shutil.copy(image_path + sorted_files[i], save_path)
    else:
        pass
