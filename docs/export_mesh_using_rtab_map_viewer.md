To export a solid textured mesh (`.obj`) directly from the graphical user interface, open your database in `rtabmap-databaseViewer` and follow this sequence:

***

### Step 1: Open the Export Window

- In the top menu bar, go to **File** > **Export 3D clouds...**. [robotics.stackexchange](https://robotics.stackexchange.com/questions/71250/how-to-install-rtab-map-in-ros-indigo)

***

### Step 2: Configure Point Density and Voxel Size

In the top section of the dialog:
- **From RGB-D images:** Ensure this box is checked. [robotics.stackexchange](https://robotics.stackexchange.com/questions/91397/it-is-possible-to-create-a-map-with-rtabmap-using-the-laser-and-obtaining-3d-poi)
- **Reconstruction flavor:** Keep as `Dense Point Cloud` (or `Organized Point Cloud` if using an organized mesh). [official-rtab-map-forum.206.s1.nabble](http://official-rtab-map-forum.206.s1.nabble.com/Scan-app-gt-import-optimize-PC-gt-export-td2634.html)
- **Assemble clouds/meshes to a single output cloud/mesh:** Keep checked. [docs.ros](http://docs.ros.org/en/lunar/api/rtabmap/html/namespacertabmap_1_1util3d.html)
- **Voxel size:** Set to `0.015 m` (or `0.010 m` for high detail).
- **Under Regenerate Clouds:** Set **3D cloud decimation** to `1` or `2` (setting to `2` preserves dense shape while keeping performance high).
- **Maximum depth / range:** Change from `4.0 m` to **`2.2 m`** (or `2.5 m` max). **Critical:** Stereo depth accuracy degrades quadratically beyond 2.5 m with a 7.5 cm camera baseline; capping this removes the wild, noisy triangles.
- **Minimum depth / range:** Set to `0.3 m` (removes near-field lens artifacts).

***

### Step 3: Enable Polygon Meshing

Scroll to the middle checkbox list:
- Check the box labeled **`Meshing.`** (the last checkbox in that group) .
- Under the newly revealed **Meshing** section:
  - **Surface reconstruction approach:** Select **`Poisson`**.
  - **Poisson depth:** Set to **`9`** or **`10`** (default `8` is too blobby/melted; `9` or `10` creates crisp, sharp furniture and wall edges).
  - **Poisson point weight:** Set to `4.0` (forces the surface to strictly hug observed points rather than puffing outward).
  - **Filter / Trim polygons (min cluster):** Set to `50` or `100` (trims away floating disconnected phantom triangles).
  - **Mesh quadric decimation factor:** Leave at `0.00` (or set a light value if you want fewer total polygons) .

***

### Step 4: Enable Texture Projection

Still within the Meshing section:
- Check the box labeled **`Texture mapping. Images of the cameras will be projected on the mesh(es)...`** .
- Under the newly revealed **Texturing** options:
  - **Texture format:** Select `.jpg` or `.png`.
  - **Output texture size:** Set to `4096x4096` (or `8192x8192` for high resolution). [github](https://github.com/introlab/rtabmap/issues)
  - **Maximum output textures:** Set to `1` (packs everything into a single texture sheet). [github](https://github.com/introlab/rtabmap/issues)
  - **Maximum distance from camera:** Set to **`2.5 m`** (or `0.0 m` if you want all surfaces painted, but setting `2.5 m` ensures low-resolution blurry photos from far away don't overwrite crisp close-up textures).
  - **Maximum camera angle:** Set to `60 deg` (avoids texturing surfaces viewed at extreme glancing angles, which look smeared).
  - **Blending:** Keep checked (smooths boundaries between overlapping camera frames).

***

### Step 5: Save as Wavefront OBJ

- Click the **Save** button in the bottom-right corner of the dialog .
- In the file browser that appears, make sure the file type dropdown is set to:
  ```text
  OBJ (*.obj)
  ```
- Choose your destination folder, enter a filename (for example, `room_mesh.obj`), and click **Save** .

***

### Output Files

RTAB-Map will process the mesh and write three companion files into that folder:
1. `room_mesh.obj` (the 3D polygon geometry)
2. `room_mesh.mtl` (material and shading definitions)
3. `room_mesh_0.png` (or `.jpg`, the compiled camera photo texture sheet) 

Keep all three files in the same directory so Blender or any 3D viewer can automatically link the image textures when loading the `.obj` .