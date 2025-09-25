from ultralytics import YOLO
import os

def train():
    # Load a model
    model = YOLO("yolo11l.pt")  # load a pretrained model (recommended for training)

    # Define log file path
    log_dir = "/home/luutunghai@gmail.com/projects/earlyFireDetection/logs"
    os.makedirs(log_dir, exist_ok=True)
    log_file = os.path.join(log_dir, "training_results.txt")

    # Train the model
    data_path = "/data/user/luutunghai@gmail.com/dataset/cv_datasets/earlyFireDetection/combined_dataset/dataset.yaml"
    results = model.train(data=data_path, epochs=50, imgsz=640)

    # Save training results to text file
    with open(log_file, 'w') as f:
        f.write("Training Results:\n")
        f.write(str(results) + "\n")
        if hasattr(results, 'results_dict'):
            f.write("\nTraining Metrics Dictionary:\n")
            for key, value in results.results_dict.items():
                f.write(f"{key}: {value}\n")

    # Evaluate the model's performance on the validation set
    metrics = model.val(save_json=True)  # save_json=True to also save detailed JSON

    # Append evaluation metrics to the text file
    with open(log_file, 'a') as f:
        f.write("\n\nEvaluation Metrics:\n")
        f.write(f"Mean Precision: {metrics.box.mp}\n")
        f.write(f"Mean Recall: {metrics.box.mr}\n")
        f.write(f"mAP@0.5: {metrics.box.map50}\n")
        f.write(f"mAP@0.5:0.95: {metrics.box.map}\n")
        # Add per-class mAP if needed
        if hasattr(metrics.box, 'maps'):
            f.write("\nPer-class mAP@0.5:0.95:\n")
            for i, map_val in enumerate(metrics.box.maps):
                class_name = metrics.names[i] if hasattr(metrics, 'names') else f"Class {i}"
                f.write(f"{class_name}: {map_val}\n")

    print(f"Training and evaluation results saved to {log_file}")


# Press the green button in the gutter to run the script.
if __name__ == '__main__':
    train()

# See PyCharm help at https://www.jetbrains.com/help/pycharm/
