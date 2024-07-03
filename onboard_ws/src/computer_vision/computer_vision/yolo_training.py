from ultralytics import YOLO
if __name__ == "__main__":
    #load model
    model = YOLO("yolov8n.yaml")  #build new model from scratch

    #use the model
    results = model.train(data='C:\\Users\\besso\\tmp\\pars.v1i.yolov8\\data.yaml', epochs=100)  #train the model