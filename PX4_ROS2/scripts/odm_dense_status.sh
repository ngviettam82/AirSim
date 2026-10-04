#!/bin/bash
# ODM dense-run status: durable products already safe on disk.
# See docs/ops-playbook.md S5.
DIR=${DIR:-$HOME/photogrammetry/bk_campus}

echo "=== container ==="
docker ps --format '{{.Image}}  {{.Status}}' || true

echo
echo "=== stage reached (last ODM stage banners) ==="
newest=$(ls -t "$DIR"/odm_dense_*.log 2>/dev/null | head -1)
echo "log: $newest"
grep -aoE "Running [a-z_0-9 ]+ stage|Finished [a-z_0-9 ]+ stage" "$newest" 2>/dev/null | tail -8
echo "--- last 3 lines ---"
tail -3 "$newest" 2>/dev/null

echo
echo "=== durable products already on disk ==="
for f in opensfm/reconstruction.json \
         openmvs/scene_dense.ply \
         odm_filterpoints/point_cloud.ply \
         odm_meshing/odm_mesh.ply \
         odm_texturing/odm_textured_model_geo.obj \
         odm_georeferencing/odm_georeferenced_model.laz \
         odm_dem/dsm.tif odm_dem/dtm.tif \
         odm_orthophoto/odm_orthophoto.tif; do
  if [ -e "$DIR/$f" ]; then
    printf "  OK      %-48s %s\n" "$f" "$(du -h "$DIR/$f" | cut -f1)"
  else
    printf "  pending %s\n" "$f"
  fi
done

echo
echo "=== resources ==="
df -h / | tail -1
free -h | head -2
