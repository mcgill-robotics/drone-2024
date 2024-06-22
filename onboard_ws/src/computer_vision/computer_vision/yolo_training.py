from ultralytics import YOLO
if __name__ == "__main__":
    #load model
    model = YOLO("C:\\Users\\besso\\tmp\\drone_2024\\onboard_ws\\src\\computer_vision\\computer_vision\\runs\\detect\\train5\\weights\\best.pt")  #build new model from scratch

    #use the model
    results = model.train(data='C:\\Users\\besso\\tmp\\shapeandhumans.v2i.yolov8\\data.yaml', epochs=100)  #train the model