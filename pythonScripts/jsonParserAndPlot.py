# pip install -r requirements.txt

import numpy as np
import matplotlib.pyplot as plt
import plotly.express as px
import pandas as pd
import json
import plotly.graph_objects as go


with open('TranslationFromIconIterativeThreshold.json', 'r') as fcc_file:
    fcc_data = json.load(fcc_file)
    #print(fcc_data)


# распаковка точек из словаря json в список списков 
toShowList = []
for i in fcc_data:
    #print(fcc_data[i]['data'])
    toShowList.append(fcc_data[i]['data'])

# список списков пихаем в таблицу
df = pd.DataFrame(data=toShowList, columns=['x', 'y', 'z'])


def plotScatter():

    x = df['x'].values
    y = df['y'].values
    z = df['z'].values


    #df = px.data.iris()
    fig = px.scatter_3d(df, x='x', y='y', z='z',
                 color=df.index       
                ,text=df.index)
    fig.show()


def plot3Dline():


    x = df['x'].values
    y = df['y'].values
    z = df['z'].values


    fig = go.Figure(data=go.Scatter3d(
        x=x, y=y, z=z,
        marker=dict(
            size=4,
            color=z,
            colorscale='Viridis',
        ),
        line=dict(
            color='darkblue',
            width=2
        )
    ))

    fig.update_layout(
        width=1600,
        height=900,
        autosize=False,
        scene=dict(
            camera=dict(
                up=dict(
                    x=0,
                    y=0,
                    z=1
                ),
                eye=dict(
                    x=0,
                    y=1.0707,
                    z=1,
                )
            ),
            aspectratio = dict( x=1, y=1, z=0.7 ),
            aspectmode = 'manual'
        ),
    )

    fig.show()

if __name__ == "__main__":
    plotScatter()
    #plot3Dline()