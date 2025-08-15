#!/usr/bin/env python3
import pandas as pd
import sys

def print_row_differences(file_path, id_column=0):
    """
    计算并打印 ODS 文件中相邻行的差值
    
    参数:
        file_path: ODS 文件路径
        id_column: 用于计算差值的列索引（默认为第 0 列）
    """
    try:
        # 读取 ODS 文件（使用 odfpy 引擎）
        df = pd.read_excel(file_path, engine="odf")
        
        if df.empty:
            print("⚠️ 文件为空！")
            return
        
        id_col = df.columns[id_column]
        print(f"🔍 正在检查列: {id_col}")
        print("行号 | 当前值 | 前一行值 | 差值")
        print("-" * 40)
        
        for i in range(1, len(df)):
            current = df.iloc[i][id_col]
            previous = df.iloc[i-1][id_col]
            
            try:
                # 尝试计算差值（适用于数字）
                diff = current - previous
                print(f"{i+1:4} | {current:6} | {previous:8} | {diff:6}")
            except (TypeError, ValueError):
                # 如果不是数字，直接显示变化（如字符串）
                print(f"{i+1:4} | {current:6} | {previous:8} | (非数字，无法计算差值)")
    
    except Exception as e:
        print(f"❌ 发生错误: {e}")

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("📌 使用方法: python print_ods_diff.py <文件.ods> [列索引]")
        print("示例: python print_ods_diff.py data.ods 0")
        sys.exit(1)
    
    file_path = sys.argv[1]
    id_column = int(sys.argv[2]) if len(sys.argv) > 2 else 0
    
    print_row_differences(file_path, id_column)
