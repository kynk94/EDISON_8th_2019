# Encoder  
<img src="https://user-images.githubusercontent.com/41245985/55304644-5717b380-5487-11e9-9086-1bef2dcaec6e.jpg" width="60%"></img>  
## 사용한 인코더 - Encoder for Pololu Wheel 42x19mm  
https://www.pololu.com/product/1217

<a href="http://www.robotoid.com/appnotes/circuits-quad-encoding.html">
 <img src="https://user-images.githubusercontent.com/41245985/55307148-9ba84c80-5491-11e9-93cf-2df2349afd03.png" width="60%" />
</a>

양쪽 바퀴의 RPM을 측정하여 PID 제어를 하기 위해 광학 인코더를 사용한다.  
각 인코더에 있는 2개의 센서에서 Wheel에 있는 12개의 톱니 요철 변화에 따라서 Pulse가 발생하며,  
이 Pulse가 단위시간에 몇회 발생하는지 감지하여 속도를 측정하였다.  
## Wheel  
#### 3D model  
<img src="https://user-images.githubusercontent.com/41245985/55305032-7283be00-5489-11e9-9e69-c1f5265dc469.PNG" width="60%"></img>  
#### Printed Wheel
<img src="https://user-images.githubusercontent.com/41245985/55305088-b676c300-5489-11e9-99d5-48cc2f524d6e.jpg" width="60%"></img>

판매처에서 Wheel은 제공하지 않고 Encoder만 제공하였으며, 또한 우리 팀 작품의 조건에 부합하지 않았다.  
때문에 Wheel의 경우 직접 모델링하여 3D 프린팅하였다.  

## 동작 영상
<a href="http://www.youtube.com/watch?feature=player_embedded&v=ZOc87ReWXjc" target="_blank">
 <img src="https://user-images.githubusercontent.com/41245985/55304215-541bc380-5485-11e9-8a2d-4a288662c4b8.png" alt="IMAGE ALT TEXT HERE" width="90%" border="10" />
</a>
