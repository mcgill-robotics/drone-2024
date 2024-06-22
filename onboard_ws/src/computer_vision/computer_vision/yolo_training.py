from ultralytics import YOLO
if __name__ == "__main__":
    #load model
    model = YOLO("yolov8n.yaml")  #build new model from scratch

    #use the model
    results = model.train(data='C:\\Users\\besso\\tmp\\shapeandhumans.v2i.yolov8\\data.yaml', epochs=1)  #train the model