# Python Version: 3.8

import pandas as pd
import numpy as np
import tensorflow as tf 

import matplotlib.pyplot as plt
import seaborn as sns
from time import sleep
import serial

sns.set(style="darkgrid")

print(tf.__version__)

# Read CSV File of Colour values
colours = pd.read_csv('colours.csv', )
#print(colours.head())

# Analyze dataset
plot_size = plt.rcParams["figure.figsize"]
plot_size[0] = 8
plot_size[1] = 6
plt.rcParams["figure.figsize"] = plot_size 

#print(colours.Colour.value_counts())

colours.Colour.value_counts().plot(kind='pie', autopct='%0.05f%%', colors=
    ['lightblue', 'lightgreen', 'orange', 'pink'], explode=(0.05, 0.05, 0.05, 0.05, 0.05))

X = pd.concat([colours.Red, colours.Green, colours.Blue], axis=1)
labels = pd.get_dummies(colours.Colour, prefix=None)
#print(labels.head)

y = labels.values 
#print(y)

output_labels = list(labels.columns)
#print(output_labels)


# Divide the dataset into training and test sets
from sklearn.model_selection import train_test_split 
X_train, X_test, y_train, y_test = train_test_split(X, y, test_size=0.2, random_state=42)

# Model Training
from tensorflow.keras.layers import Input, Dense, Activation, Dropout
from tensorflow.keras.models import Model 

# Create Classification Model
input_layer = Input(shape=(X.shape[1],))
dense_layer_1 = Dense(15, activation='relu')(input_layer)
dense_layer_2 = Dense(10, activation='relu')(dense_layer_1)
output = Dense(y.shape[1], activation='softmax')(dense_layer_2)

create_model = False
useSerial = True
usePipe = False

if create_model:
    model = Model(inputs=input_layer, outputs=output)
    model.compile(loss='categorical_crossentropy', optimizer='adam', metrics=['acc'])

    #print(model.summary())

    # Train Model
    history = model.fit(X_train, y_train, batch_size=4, epochs=1250, verbose=1, validation_split=0.2)

    # Evaluate Performance
    score = model.evaluate(X_test, y_test, verbose=1)
    print("Test Score: ", score[0])
    print("Test Accuracy: ", score[1])

    model.save('saved_model/colours_model')

else:
    model = tf.keras.models.load_model('saved_model/colours_model')
    #model.summary()
    #print(X_test)
    
    if useSerial:
    	ser = serial.Serial('com8', 115200) # establish connection with microcontroller
    
    elif usePipe:
	    fifo_path = "/tmp/fifopipe"
	
    while True:

        if useSerial:
            rgb = str(ser.readline())[2:-5] # remove \r\n
        elif usePipe:
            fifo = open(fifo_path, 'r')
            rgb = fifo.read()[:-2] # remove \r\n

        sleep(0.5)
        r, g, b = rgb.split(",")
        #print(r,g,b)
        r = int(r)
        g = int(g)
        b = int(b)

        prediction = model.predict(np.array([[r, g, b]]))
        #print(prediction.shape)
        #print(prediction)
        max_val = np.amax(prediction)
        max_idx = np.where(prediction == np.amax(prediction))[1]
        max_idx = max_idx[0]
        print(f"RAW RGB: {r}, {g}, {b}")
        print(f"Max value: {max_val}% at index: {max_idx}")
        print(f"Colour Predicted ({max_val}%): {output_labels[max_idx]}")

        if usePipe:
            fifo.close()
