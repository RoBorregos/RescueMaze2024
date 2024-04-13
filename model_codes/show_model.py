
import torch
import torch.nn as nn
import torch.optim as optim
from torch.optim import lr_scheduler
import torch.backends.cudnn as cudnn
import numpy as np
import torchvision
from torchvision import datasets, models, transforms
import matplotlib.pyplot as plt
import time
import os
from PIL import Image
from tempfile import TemporaryDirectory


data_transforms = {
    'train': transforms.Compose([
        transforms.ToTensor(),
    ]),
    'val': transforms.Compose([
        transforms.ToTensor(),

    ]),
}


data_dir = 'dataset_two/'
image_datasets = {x: datasets.ImageFolder(os.path.join(data_dir, x), data_transforms[x]) for x in ['train', 'val']}
dataloaders = {x: torch.utils.data.DataLoader(image_datasets[x], batch_size=4,shuffle=True, num_workers=4)for x in ['train', 'val']}
print(dataloaders)
dataset_sizes = {x: len(image_datasets[x]) for x in ['train', 'val']}
class_names = image_datasets['train'].classes
print(class_names)

inputs, classes = next(iter(dataloaders['train']))
out = torchvision.utils.make_grid(inputs)
#imshow(out, title=[class_names[x] for x in classes])
device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
print(device)


def visualize_model(model, num_images=6):
    was_training = model.training
    model.eval()
    images_so_far = 0
   
    with torch.no_grad():
        for i, (inputs, labels) in enumerate(dataloaders['val']):
            inputs = inputs.to(device)
            labels = labels.to(device)

            outputs = model(inputs)
            _, preds = torch.max(outputs, 1)

            for j in range(inputs.size()[0]):
                images_so_far += 1
                ax = plt.subplot(num_images//2, 2, images_so_far)
                ax.axis('off')
                ax.set_title(f'predicted: {class_names[preds[j]]}')
                #imshow(inputs.cpu().data[j])

                if images_so_far == num_images:
                    model.train(mode=was_training)
                    return
        model.train(mode=was_training)

def test_model(model,num_images=5):
    model.eval()
    images_so_far = 0
    print(dataloaders['val'])

def predict_image(model, image_path):
    # Define the image transformation
    transform = transforms.Compose([
        transforms.ToTensor(),
    ])

    # Load the image and apply the transformation

    image = Image.open(image_path)
    image = Image.open(image_path).convert("RGB")
    image = transform(image).unsqueeze(0)

    # Move the image to the device
    image = image.to(device)

    # Set the model to evaluation mode
    model.eval()

    # Predict the class of the image
    print("started")
    time1 = time.time()
    with torch.no_grad():
        output = model(image)
        _, predicted = torch.max(output, 1)
    print(time.time()-time1)
    return class_names[predicted]
  # pause a bit so that plots are updated

model_ft = models.resnet18(weights='IMAGENET1K_V1')
num_ftrs = model_ft.fc.in_features
model_ft.fc = nn.Linear(num_ftrs, len(class_names))
model_ft = model_ft.to(device)
model_ft.load_state_dict(torch.load('best_model_params.pt'))
print(predict_image(model_ft,"dataset_two/val/h/400.jpg"))