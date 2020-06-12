# 运行约需10s
#### 标准库
from collections import defaultdict
#### 第三方库
import numpy as np


#### 定义常数
n = 0 # 上车点个数
l_c = 5.00 # 车辆长度
d_c = 20.00 # 上车点间距
t_s = 30.00 # 服务时间期望值为30s
v = 30/3.6 # 安全车速
t = 0.01 # 模拟精度为 0.01s

#### 定义全局变量
current_code = 1 # 为每一辆入场的出租车编号，每次加一
conflicts = set() # 记录已发生的冲突
remaining_service_time = {} # 每个上车点此刻剩余的服务时间
cars_on_road = {} # 键为目标上车点，值为目前所在位置，若已载客，目标上车点标为0
end_time = 1000 # 模拟终止时间


#### 定义函数
# 根据指数分布随机生成服务时间
def random_service_time(t_s=t_s):
    return np.random.exponential(t_s)

# 模拟情境初始化
def init_start(N, v=v,d=d_c):
    global current_code, conflicts
    conflicts = set()
    for idx in range(N):
        # idx+1同时为第一批车每辆的编号
        current_code = idx + 1
        remaining_service_time[idx] = [random_service_time(), current_code]

    return remaining_service_time

# 开始模拟
# 假设：每辆车起点距离为0，第一个上车点距离为 l_c（车辆长度）
def simulate(N, l_road, d=d_c, t=t, v=v):
    global conflicts, current_code
    remaining_service_time = init_start(N=N)
    current_time = 0
    while True:
        # 若达到模拟时间，返回结果
        if current_time >= end_time:
            return current_code - len(conflicts)
        # 每次推进单位时间（0.01s）
        current_time += t
        # 更新所有车辆距离
        for k in cars_on_road.keys():
            # 若不是已载客车
            if k != 0:
                try:
                    cars_on_road[k][0] += v * t
                except: 
                    pass
            # 若是已载客车
            else:
                if k in cars_on_road.keys():
                    for car in cars_on_road[k]:
                        if type(car) == list:
                            car[0] += v * t
                        else:
                            cars_on_road[k].remove(car)
                else:
                    pass

        for k in remaining_service_time.keys():
            if type(remaining_service_time[k][0]) == float:
                # 剩余服务时间相应减少
                remaining_service_time[k][0] -= t

        # 将已驶离出候车区的车移出路面
        remove_out_cars(l_road)
        # 将已完成服务的车移回行驶通道，并发派补位车
        done_service()
        # 将到达目标上车点的补位车填入服务车道
        move_cars_to_service()
        # 计算每辆车是否发生危险会车事件
        calculate_conflicts()
    return

# 更新事件：出租车驶离候车区
def remove_out_cars(l_road):
    # 若道路上无载客车
    if 0 not in cars_on_road.keys():
        return
    else:
        # 对所有已载客车
        for car in cars_on_road[0]:
            if type(car) == list:
                if car[0] >= l_road: # 当距离大于通道长度时
                    # 将车移出通行通道
                    cars_on_road[0].remove(car)
                # print(car)
        if len(cars_on_road[0]) == 0: del(cars_on_road[0])
    return

# 更新事件：出租车到达目标上车点，开始服务
def move_cars_to_service(d_c=d_c,t_s=t_s):
    for k in list(cars_on_road.keys()):
        if k == 0: # 跳过已载客，正在驶离的车 
            continue
        else:
            v = cars_on_road[k]
            if v[0] >= l_c + d_c * (k - 1): # 若在0.01s内，该车到达服务点
                del(cars_on_road[k]) # 将该车移出通行通道
                # 更新该上车点服务时间，及正在服务的车的编号
                remaining_service_time[k] = [random_service_time(), v[1]]
    return

# 更新事件：服务完成
def done_service(l_c=l_c, d_c=d_c):
    global current_code
    undefined = 99.99 # 代表上车点空闲中
    for k, v in remaining_service_time.items():
        # v[0]为剩余服务时间，v[1]为出租车编号  
        if v[1] != 0:  
            try:      
                if v[0] <= 0:        
                    # 如果路上没车
                    if any_car_on_road() == False:
                        remaining_service_time[k] = [undefined, 0]
                        current_code += 1
                        cars_on_road[k] = [0, current_code]
                    else:
                        # 最后发车的车离起点距离
                        last_car = undefined
                        # 更新k点服务状态：空闲中
                        remaining_service_time[k] = [undefined, 0]
                        # 载客完成的车开始驶离
                        if 0 in cars_on_road.keys():
                            cars_on_road[0].append(  [l_c + d_c, v[1]]  )
                        else:
                            cars_on_road[0] =[  [ l_c + d_c, v[1] ]  ]
                            # 若此时上一辆发车还未完成（即驶出l_c/v距离）
                            if last_car <= 0:
                                current_code += 1
                                # 则此辆车排在上一辆车后发车
                                cars_on_road[k] = [last_car - l_c, current_code]
                            # 若上一辆车已完成发车
                            else:
                                current_code += 1
                                # 则直接从起点发车
                                cars_on_road[k] = [0, current_code]
            except: pass
    return

# 更新事件：路上每辆车是否发生危险会车事件
def calculate_conflicts(l_c=l_c):
    global conflicts
    heading_cars = [] # 已载客车列
    leaving_cars = [] # 未载客车列
    for goal, car in cars_on_road.items():
        if goal != 0:
            if len(car) > 1: heading_cars.append(car)
        else:
            if len(cars_on_road[goal]) > 0:
                leaving_cars = cars_on_road[goal]

    for c1 in heading_cars:
        d1, code1 = c1[0], c1[1]
        for c2 in leaving_cars:
            try:
                d2, code2 = c2[0], c2[1]
                # 距离过近则用集合记录发生
                if abs(d1 - d2) < l_c:
                    conflicts.add(tuple((code1, code2)))
            except: pass
    return

def any_car_on_road():
    for k in cars_on_road.keys():
        if len(cars_on_road[k]) > 0:
            return True
    return False


if __name__ == "__main__":
    results = []
    for n in range(2, 20):
        # 返回评价函数
        l_road = l_c + (n-1) * d_c # 候车区道路长度
        current_code = 0
        goodness = simulate(n, l_road)
        # 存储评价结果
        results.append(tuple((goodness, n)))
    # 从好到坏进行排序
    results.sort(reverse=True)
    print(results[0][1])