import pandas as pd

# 读取 CSV 文件
df = pd.read_csv('/home/wxr/proj/traversability/obstacle_detection/ws_obdt/src/obstacle_mapping/test_map/6t_hight.csv')

# 获取行数和列数
num_rows, num_cols = df.shape

print(f"行数: {num_rows}, 列数: {num_cols}")
