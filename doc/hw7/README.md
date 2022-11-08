# 作业 7（极小曲面全局方法、曲面参数化）  

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

**实现极小曲面的全局方法：边界固定，求解方程组**  

- 点击 `Mesh to HEMesh`  
- 点击 `Minimal Surface`  
- 点击 `HEMesh to Mesh`  

`Balls`：  

![balls_minisurf](images/balls_minisurf.jpg)

`David328`：  

![david328_minisurf](images/david328_minisurf.jpg)

`Nefertiti_face`：  

![nefertiti_face_minisurf](images/nefertiti_face_minisurf.jpg)

**实现曲面参数化：边界映射到平面，求解方程组**  

- 点击 `Mesh to HEMesh`  
- 点击 `Parameterization`  
- 点击 `HEMesh to Mesh`  

`Balls`：  

![balls_para](images/balls_para.jpg)

`David328`：  

![david328_para](images/david328_para.jpg)

`Nefertiti_face`：  

![nefertiti_face_para](images/nefertiti_face_para.jpg)