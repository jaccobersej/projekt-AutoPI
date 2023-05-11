import tensorflow as tf
#import keras 
import numpy as np
from PIL import Image
import matplotlib.pyplot as plt
import time
import glob
import matplotlib.cm as cm
import os

#Load model
model = tf.keras.models.load_model("dumb/models")

#Definer navn på sidste lag i model
last_layer = "conv2d_4"

#Billed dimensioner
image_dimensions = (600, 800)
fov_x = 48
fov_y = 64
angle_pr_pixel = fov_x/image_dimensions[0]

#Højder der skal bruges til trigonometri senere
average_height = 180
height_of_camera = 10

#genererer en heatmap af et givent billede ud fra aktivationerne af det sidste lag af convolution.
def gradcam_heatmap(image, model, last_convolution):
    grad_model = tf.keras.models.Model([model.inputs], [model.get_layer(last_convolution).output, model.output])
    with tf.GradientTape() as tape:
        last_convolution_output, preds = grad_model(image)
        pred_index = np.argmax(preds[0])
        class_channel = preds[:, pred_index]

    grads = tape.gradient(class_channel, last_convolution_output)
    pooled_grads = tf.reduce_mean(grads, axis=(0, 1, 2))
    last_convolution_output = last_convolution_output[0]
    heatmap = last_convolution_output @ pooled_grads[..., tf.newaxis]
    heatmap = tf.squeeze(heatmap)
    heatmap = tf.maximum(heatmap, 0) / tf.math.reduce_max(heatmap)

    return heatmap.numpy()


#Mens den kører:
while 1:
    st = time.time()
    images = glob.glob("images/*.jpg")
    #Hvis der er nogle billeder:
    if len(images) > 0:
        time.sleep(0.05)
        try:
            listOfImages = np.array([np.array(Image.open(image).transpose(method=2)) for image in images])
        except:
            pass
        #Forudsig på billederne
        test = model.predict(listOfImages)
        a = np.average(test, axis=0)
        #Hvis målets average er over threshold værdien, hvilket er en sandsynlighedsvurdering på 75%:
        if a[1] >= 0.75:
            while 1:
                #åben første billede
                image = np.array([np.array(Image.open("images/a0.jpg").transpose(method=2))])
                test = model.predict(image)
                print("hej",test[0])
                #Hvis billedet havde målet på:
                if test[0][1] >= 0.75:
                    #Lav heatmap
                    heatmap = gradcam_heatmap(image, model, last_layer)
                    colormap = np.uint8(heatmap*255)
                    jet = cm.get_cmap("jet")
                    jet_colors = jet(np.arange(256))[:, :3]
                    jet_heatmap = jet_colors[colormap]
                    
                    #Beregn den gennemsnitlige position af koordinaterne på heatmappen
                    koord_x = []
                    koord_y = []
                    for x in range(0, jet_heatmap.shape[0]):
                        for y in range(0, jet_heatmap.shape[1]):
                            for z in range(0, jet_heatmap.shape[2]):
                                if jet_heatmap[x, y, z] >= 0.8:
                                    koord_x.append(x)
                                    koord_y.append(y)
                    average_x = np.sum(koord_x)/len(koord_x)*16.1
                    average_y = np.sum(koord_y)/len(koord_y)*16.1

                    #Beregn afstand og vinkel til målet.
                    angle_from_camera_x = (average_x-image_dimensions[0]/2)*angle_pr_pixel
                    angle_from_x_axis = angle_from_camera_x+30
                    distance = (average_height-height_of_camera)/np.tan(np.deg2rad(angle_from_x_axis))

                    angle_from_camera_y = (average_y-image_dimensions[1]/2)*angle_pr_pixel
                    distance = distance/np.cos(np.deg2rad(angle_from_camera_y))
                    
                    vektor_x = distance * np.cos(np.deg2rad(angle_from_camera_y))
                    vektor_y = distance * np.sin(np.deg2rad(angle_from_camera_y))

                    file2 = open("Vektor.txt", "w+")
                    file2.write(str(round(vektor_x, 2))+","+str(round(vektor_y, 2)))
                    file2.close()

                break
        
        #Gem resultaterne i en txt fil, som de andre scripts så kan læse
        file1 = open("Guess.txt", "w+")
        file1.write(str(round(a[0], 2))+","+str(round(a[1], 2)))
        file1.close()
        #DEBUGGING /// print("Andre: %.2f" % a[0], "Hector: %.2f" % a[1], "Intet ansigt: %.2f" % a[2], "Jacob: %.2f" % a[3], "Marcus: %.2f" % a[4])
        
        #Hvor lang tid tog det? Lad os printe det
        print(time.time()-st)
        
        #Slet alle billederne
        time.sleep(0.05)
        for image in images:
            os.remove(image)