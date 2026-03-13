```python
static void YDIFlyServoSinControl( float l_angle_max, float l_angle_min, float r_angle_max, float r_angle_min, float T, float speed_diff )
{
    float angle_set = 0;
    static float time_now = 0;

    /* 输入参数保护 */
    if( speed_diff > YDIFLY_CONTROL_CYCLE )         speed_diff = YDIFLY_CONTROL_CYCLE-1;
    else if( speed_diff < -YDIFLY_CONTROL_CYCLE )   speed_diff =-YDIFLY_CONTROL_CYCLE+1;
    // (l_angle_max-l_angle_min)/2 将标准正弦函数的幅值放大到所需的扑翼震动角度
    // (l_angle_max+l_angle_min)/2 将标准正弦函数的中性点水平平移
    // sin( time_now*6.283185307179586/T )生成目前时间下的[-1,1]的系数
    angle_set = ((l_angle_max-l_angle_min)/2)*sin( time_now*6.283185307179586/T ) + (l_angle_max+l_angle_min)/2;
    YDIFlyServoAngleControl( SERVO_L, angle_set );

    angle_set = ((r_angle_max-r_angle_min)/2)*sin( time_now*6.283185307179586/T ) + (r_angle_max+r_angle_min)/2;
    YDIFlyServoAngleControl( SERVO_R, angle_set );

    time_now += YDIFLY_CONTROL_CYCLE;
    if( (time_now > T*0.25f) && (time_now < T*0.75f) )  time_now -= speed_diff;
    else                                                time_now += speed_diff;

    if( time_now > T )
    {
        time_now -= T;
    }    
}
```

是实现**平滑往复运动正弦波运动**的标准数学公式。它的核心作用是将一个范围在 $[-1, 1]$ 之间的正弦波，映射到你设定的舵机运动范围 $[Angle_{min}, Angle_{max}]$ 内。

我们可以将其拆解为数学上的 **$y = A \sin(\theta) + B$** 函数。

---

### 1. 数学结构拆解

公式可以整理为：


$$Angle_{set} = \underbrace{\frac{Angle_{max} - Angle_{min}}{2}}_{\text{振幅 (Amplitude)}} \cdot \sin\left( \frac{Time \cdot 2\pi}{T} \right) + \underbrace{\frac{Angle_{max} + Angle_{min}}{2}}_{\text{中心偏移 (Vertical Shift)}}$$

#### A. 振幅部分：$\frac{Angle_{max} - Angle_{min}}{2}$

* **原理**：`sin` 函数的输出值在 $[-1, 1]$ 之间，范围跨度是 $2$。为了让输出的范围刚好等于 `max` 到 `min` 的距离（即 $Angle_{max} - Angle_{min}$），我们需要将 `sin` 的结果乘以这个系数。
* **物理意义**：这代表翅膀摆动的“力度”或“行程半径”。

#### B. 中心偏移部分：$\frac{Angle_{max} + Angle_{min}}{2}$

* **原理**：原来的 `sin` 函数是以 $0$ 为中心震荡的（在 $-1$ 到 $1$ 之间）。通过加上这个平均值，我们将整个震荡曲线向上（或向下）平移，使得震荡的中心点刚好落在两个极限角度的中间。
* **物理意义**：这代表翅膀摆动的“平衡中立点”。

#### C. 相位角部分：$\frac{time\_now \cdot 6.283185307179586}{T}$

* **原理**：这是正弦函数的自变量 $\theta$。
* `6.283185307179586` 就是 $2\pi$（即一个圆周的弧度）。
* `time_now / T` 计算的是当前时刻在整个周期 $T$ 中所处的进度百分比。
* 相乘后得到的是弧度制角度。这确保了每经过时间 $T$，函数刚好完成一个完整的 $2\pi$ 周期（从 0 变到 $2\pi$）。



---

### 2. 为什么蝴蝶机要用这个公式？

如果直接让舵机在 `max` 和 `min` 之间做“线性跳变”（例如 0 度到 90 度瞬间切换），翅膀会发生剧烈震动，不仅由于惯性导致机械结构（齿轮）极易损毁，蝴蝶也根本飞不起来。

**使用正弦函数的好处：**

1. **速度平滑（Smooth Velocity）**：正弦运动在波峰和波谷处速度为 0，在中间处速度最快。这意味着翅膀在到达极限位置换向时是**减速停止再加速**的，而不是猛烈撞击。
2. **符合空气动力学**：自然的鸟类和昆虫扑翼并非生硬的“上下切换”，而是一种柔和的挥舞动作。正弦波能模拟这种自然的流体动力学特性。
3. **降低机械磨损**：减少了舵机和连杆在换向瞬间的瞬时冲击力，保护硬件寿命。

### 总结

这行代码通过简单的三角函数变换，将抽象的“时间流”转换成了蝴蝶翅膀优雅的“摆动幅度”。

* **$time\_now$** 是节拍器。
* **$T$** 是扑翼的频率。
* **$Angle_{max}$ 和 $Angle_{min}$** 是物理限制。

**如果你把公式里的 `sin` 换成 `cos`，你知道会发生什么吗？**（提示：蝴蝶翅膀初始位置会发生变化，从“最开”或“最闭”开始运动。）