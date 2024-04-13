from PIL import Image

image = Image.open("vision/hModels/H.jpg")

for i in range(0, 360, 30):
    image = image.convert("L")
    image = Image.eval(image, lambda pixel: 255 - pixel)
    image = image.rotate(i)
    image = image.convert("L")
    image = Image.eval(image, lambda pixel: 255 - pixel)
    image.save(f"vision/hModels/rotated_H_{i}.jpg")