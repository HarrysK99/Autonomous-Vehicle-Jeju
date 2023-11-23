import numpy as np

# 사용자로부터 두 개의 좌표값 입력받기
x1, y1 = map(float, input("첫 번째 좌표를 입력하세요 (x,y): ").split(","))
x2, y2 = map(float, input("두 번째 좌표를 입력하세요 (x,y): ").split(","))

# 두 포인트 사이에 648개의 균일한 좌푯값 생성
x = np.linspace(x1, x2, 649)[1:-1]
y = np.linspace(y1, y2, 649)[1:-1]

# 좌표값을 a.txt 파일에 저장
with open("a.txt", "w") as f:
    for i in range(len(x)):
        f.write(f"{x[i]:.6f},{y[i]:.6f}\n")
