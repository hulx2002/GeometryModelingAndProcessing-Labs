# 作业 6（极小曲面局部方法）  

**输入网格**  

- 在 `Hierarchy` 中右键空白处，选择 `Create Empty Entity`  
- 在 `Inspector` 中点击 `Attach Component`，选择 `struct Ubpa::Utopia::LocalToWorld`、`struct Ubpa::Utopia::MeshFilter` 和 `struct Ubpa::Utopia::MeshRenderer`  
- 将 `Project&Folder` 中的 `assets/models/Balls` 拖入 `mesh`，`assets/materials/wireframe` 拖入 `materials`  
- 在 `Hierarchy` 中点击 `Denoise Data`，将 `assets/models/Balls` 拖入 `mesh`  
- 可将 `Balls` 替换为其它网格，如 `David328` 和 `Nefertiti_face`  

![prepare](images/prepare.jpg)

`Balls`：  

![balls](images/balls.jpg)

`David328`：  

![david328](images/david328.jpg)

`Nefertiti_face`：  

![nefertiti_face](images/nefertiti_face.jpg)

**实现极小曲面的局部法**  

- 点击 `Mesh to HEMesh`  
- 输入迭代次数 $iterations$  
- 输入参数 $\lambda=0.01$  
- 点击 `Minimal Surface`  
- 点击 `HEMesh to Mesh`  

`Balls`：  

$iterations=250$  

![balls_250](images/balls_250.jpg)

$iterations=500$  

![balls_500](images/balls_500.jpg)

$iterations=750$  

![balls_750](images/balls_750.jpg)

$iterations=1000$  

![balls_1000](images/balls_1000.jpg)

`David328`：  

$iterations=250$  

![david328_250](images/david328_250.jpg)

$iterations=500$  

![david328_500](images/david328_500.jpg)

$iterations=750$  

![david328_750](images/david328_750.jpg)

$iterations=1000$  

![david328_1000](images/david328_1000.jpg)

`Nefertiti_face`：  

$iterations=250$  

![Nefertiti_face_250](images/Nefertiti_face_250.jpg)

$iterations=500$  

![Nefertiti_face_500](images/Nefertiti_face_500.jpg)

$iterations=750$  

![Nefertiti_face_750](images/Nefertiti_face_750.jpg)

$iterations=1000$  

![Nefertiti_face_1000](images/Nefertiti_face_1000.jpg)