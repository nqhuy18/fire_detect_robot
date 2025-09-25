import zipfile
import os
import shutil
import yaml
from sklearn.model_selection import train_test_split
import glob
from collections import defaultdict
from PIL import Image, ImageDraw, ImageFont
import random


def extract_data():
    # Root path where the zip files are located
    root_path = "/data/user/luutunghai@gmail.com/dataset/cv_datasets/earlyFireDetection"

    # Find all relevant zip files in the root_path
    zip_files = glob.glob(os.path.join(root_path, 'Test_*.v1i.yolov11.zip'))

    # Output directory for the combined dataset, placed within the root_path
    output_dir = os.path.join(root_path, 'combined_dataset')
    if os.path.exists(output_dir):
        shutil.rmtree(output_dir)
    os.makedirs(output_dir, exist_ok=True)

    # Global class management
    class_id_map = {}
    global_names = []

    # List to hold all image-label pairs
    all_images = []

    # Temporary directories
    temp_dirs = []

    for zip_file in zip_files:
        # Create a temporary directory for extraction within root_path
        temp_dir = os.path.join(root_path, f'temp_{os.path.splitext(os.path.basename(zip_file))[0]}')
        os.makedirs(temp_dir, exist_ok=True)
        temp_dirs.append(temp_dir)

        # Extract the zip
        with zipfile.ZipFile(zip_file, 'r') as z:
            z.extractall(temp_dir)

        # Load data.yaml
        data_yaml_path = os.path.join(temp_dir, 'data.yaml')
        with open(data_yaml_path, 'r') as f:
            data = yaml.safe_load(f)

        # Get local class names and normalize them (replace '-' with '_', spaces with '_', lowercase)
        local_names = []
        for name in data['names']:
            normalized_name = name.lower().replace('-', '_').replace(' ', '_')
            local_names.append(normalized_name)

        # Map local classes to global IDs
        for local_id, local_name in enumerate(local_names):
            if local_name not in class_id_map:
                global_names.append(local_name)
                class_id_map[local_name] = len(global_names) - 1

        # Process labels: remap class IDs
        labels_dir = os.path.join(temp_dir, 'train', 'labels')
        for label_file in os.listdir(labels_dir):
            label_path = os.path.join(labels_dir, label_file)
            with open(label_path, 'r') as f:
                lines = f.readlines()

            new_lines = []
            for line in lines:
                parts = line.strip().split()
                if parts:
                    local_id = int(parts[0])
                    global_id = class_id_map[local_names[local_id]]
                    parts[0] = str(global_id)
                    new_lines.append(' '.join(parts) + '\n')

            with open(label_path, 'w') as f:
                f.writelines(new_lines)

        # Collect images and corresponding labels (include all images, even without labels)
        images_dir = os.path.join(temp_dir, 'train', 'images')
        for img_file in os.listdir(images_dir):
            if img_file.lower().endswith(('.jpg', '.jpeg', '.png')):
                img_path = os.path.join(images_dir, img_file)
                label_name = os.path.splitext(img_file)[0] + '.txt'
                label_path = os.path.join(labels_dir, label_name)
                if os.path.exists(label_path):
                    # Check if label file is empty
                    if os.path.getsize(label_path) > 0:
                        all_images.append((img_path, label_path))
                    else:
                        all_images.append((img_path, None))
                else:
                    all_images.append((img_path, None))

    # Split into train and val (80/20 split)
    if all_images:
        train_imgs, val_imgs = train_test_split(all_images, test_size=0.2, random_state=42)
    else:
        print("No images found.")
        train_imgs = []
        val_imgs = []

    # Create output directories
    images_train_dir = os.path.join(output_dir, 'images', 'train')
    images_val_dir = os.path.join(output_dir, 'images', 'val')
    labels_train_dir = os.path.join(output_dir, 'labels', 'train')
    labels_val_dir = os.path.join(output_dir, 'labels', 'val')

    os.makedirs(images_train_dir, exist_ok=True)
    os.makedirs(images_val_dir, exist_ok=True)
    os.makedirs(labels_train_dir, exist_ok=True)
    os.makedirs(labels_val_dir, exist_ok=True)

    # Copy files to train directories
    for img_path, label_path in train_imgs:
        shutil.copy(img_path, os.path.join(images_train_dir, os.path.basename(img_path)))
        if label_path is not None:
            shutil.copy(label_path, os.path.join(labels_train_dir, os.path.basename(label_path)))

    # Copy files to val directories
    for img_path, label_path in val_imgs:
        shutil.copy(img_path, os.path.join(images_val_dir, os.path.basename(img_path)))
        if label_path is not None:
            shutil.copy(label_path, os.path.join(labels_val_dir, os.path.basename(label_path)))

    # Create unified dataset.yaml with names as dict
    names_dict = {i: name for i, name in enumerate(global_names)}
    dataset_yaml = {
        'nc': len(global_names),
        'names': names_dict,
        'train': 'images/train',
        'val': 'images/val'
    }

    with open(os.path.join(output_dir, 'dataset.yaml'), 'w') as f:
        yaml.dump(dataset_yaml, f, sort_keys=False)

    # Clean up temporary directories
    for temp_dir in temp_dirs:
        shutil.rmtree(temp_dir)

    print(
        f"Combined dataset created in '{output_dir}' with {len(train_imgs)} train samples and {len(val_imgs)} val samples.")
    print(f"Classes: {global_names}")


def data_statistics():
    root_path = "/data/user/luutunghai@gmail.com/dataset/cv_datasets/earlyFireDetection"
    output_dir = os.path.join(root_path, 'combined_dataset')

    # Load dataset.yaml
    yaml_path = os.path.join(output_dir, 'dataset.yaml')
    if not os.path.exists(yaml_path):
        print(f"Dataset YAML not found at {yaml_path}")
        return None

    with open(yaml_path, 'r') as f:
        data = yaml.safe_load(f)

    num_classes = data.get('nc', 0)
    class_names = data.get('names', [])

    if num_classes != len(class_names):
        print("Mismatch between nc and names in dataset.yaml")
        return None

    # Collect all label files from train and val
    label_dirs = [
        os.path.join(output_dir, 'labels', 'train'),
        os.path.join(output_dir, 'labels', 'val')
    ]

    images_per_class = defaultdict(set)  # class_id -> set of image basenames

    total_labels_found = 0
    for label_dir in label_dirs:
        if not os.path.exists(label_dir):
            continue
        for label_file in os.listdir(label_dir):
            if label_file.endswith('.txt'):
                total_labels_found += 1
                image_basename = os.path.splitext(label_file)[0]
                label_path = os.path.join(label_dir, label_file)
                try:
                    with open(label_path, 'r') as f:
                        for line in f:
                            if line.strip():
                                parts = line.split()
                                class_id = int(parts[0])
                                if 0 <= class_id < num_classes:
                                    images_per_class[class_id].add(image_basename)
                                else:
                                    print(f"Warning: Invalid class_id {class_id} in {label_path}")
                except Exception as e:
                    print(f"Error reading {label_path}: {e}")

    # Compute counts
    counts = {class_names[i]: len(images_per_class[i]) for i in range(num_classes)}

    # Total unique images (across all classes)
    all_images = set()
    for img_set in images_per_class.values():
        all_images.update(img_set)
    total_images = len(all_images)

    # Output the information
    print(f"Number of classes: {num_classes}")
    print("Class names:", class_names)
    print("Images per class:")
    for name, count in counts.items():
        print(f"  - {name}: {count} images")
    print(f"Total unique images: {total_images}")
    print(f"Total label files processed: {total_labels_found}")

    # Return for potential use
    return {
        'num_classes': num_classes,
        'class_names': class_names,
        'images_per_class': counts,
        'total_images': total_images
    }


def visualize_random_bboxes(num_images=10):
    """
    Function to select 10 random images from the combined_dataset (train + val),
    draw bounding boxes based on their YOLO labels, and save them to a visualization folder.
    This helps visually check if the bounding boxes are correct, especially on HPC where display is not possible.
    """
    root_path = "/data/user/luutunghai@gmail.com/dataset/cv_datasets/earlyFireDetection"
    output_dir = os.path.join(root_path, 'combined_dataset')

    # Visualization save path
    viz_path = "/home/luutunghai@gmail.com/projects/earlyFireDetection/visualization"
    os.makedirs(viz_path, exist_ok=True)

    # Load class names from dataset.yaml
    yaml_path = os.path.join(output_dir, 'dataset.yaml')
    if not os.path.exists(yaml_path):
        print(f"Dataset YAML not found at {yaml_path}")
        return

    with open(yaml_path, 'r') as f:
        data = yaml.safe_load(f)
    class_names = data.get('names', [])

    # Collect all image paths from train and val
    image_dirs = [
        os.path.join(output_dir, 'images', 'train'),
        os.path.join(output_dir, 'images', 'val')
    ]

    all_images = []
    for img_dir in image_dirs:
        if os.path.exists(img_dir):
            all_images.extend([
                os.path.join(img_dir, f) for f in os.listdir(img_dir)
                if f.lower().endswith(('.jpg', '.jpeg', '.png'))
            ])

    if not all_images:
        print("No images found in the dataset.")
        return

    # Sample random images
    num_images = min(num_images, len(all_images))
    selected_images = random.sample(all_images, num_images)

    saved_files = []
    for img_path in selected_images:
        # Load image
        img = Image.open(img_path).convert("RGB")
        draw = ImageDraw.Draw(img)
        img_width, img_height = img.size

        # Try to load a font (use default if not available)
        try:
            font = ImageFont.truetype("arial.ttf", 15)
        except IOError:
            font = ImageFont.load_default()

        # Construct corresponding label path
        relative_path = os.path.relpath(img_path, os.path.join(output_dir, 'images'))
        label_subdir = os.path.dirname(relative_path)
        label_dir = os.path.join(output_dir, 'labels', label_subdir)
        label_name = os.path.splitext(os.path.basename(img_path))[0] + '.txt'
        label_path = os.path.join(label_dir, label_name)

        # Draw bounding boxes if label exists
        if os.path.exists(label_path):
            with open(label_path, 'r') as f:
                for line in f:
                    parts = line.strip().split()
                    if len(parts) == 5:
                        try:
                            class_id = int(parts[0])
                            x_center, y_center, width, height = map(float, parts[1:])

                            # Convert normalized to absolute coordinates
                            x1 = (x_center - width / 2) * img_width
                            y1 = (y_center - height / 2) * img_height
                            x2 = (x_center + width / 2) * img_width
                            y2 = (y_center + height / 2) * img_height

                            # Draw rectangle
                            draw.rectangle([x1, y1, x2, y2], outline="red", width=3)

                            # Add class name text (if class_id is valid)
                            if 0 <= class_id < len(class_names):
                                draw.text((x1, y1 - 15), class_names[class_id], fill="red", font=font)
                        except ValueError:
                            print(f"Invalid label format in {label_path}")
        else:
            print(f"No label found for {img_path}")

        # Save the visualized image
        save_name = f"visualized_{os.path.basename(img_path)}"
        save_path = os.path.join(viz_path, save_name)
        img.save(save_path)
        saved_files.append(save_path)

    print(f"Saved {len(saved_files)} visualized images to {viz_path}:")
    for file in saved_files:
        print(f" - {file}")


if __name__ == "__main__":
    extract_data()
    data_statistics()
    visualize_random_bboxes()