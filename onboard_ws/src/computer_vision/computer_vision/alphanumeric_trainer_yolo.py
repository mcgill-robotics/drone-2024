from ultralytics import YOLO

#load model
model = YOLO("yolov8n.yaml") #build new model from scratch

#use the model
results = model.train(data='data.yaml', epochs=20) #train the model