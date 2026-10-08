import * as THREE from 'three';
import { novaCADMaterials } from './nova-cad-materials.js';

export function novaMaterialForMesh(url, fallback) {
  const name=decodeURIComponent(url.split(/[?#]/)[0].split('/').pop()).replace(/\.stl$/i,'');
  const source=novaCADMaterials[name];
  if(!source)return fallback;
  const [r,g,b,alpha=1]=source.baseColorFactor;
  return new THREE.MeshStandardMaterial({
    name:`NOVA CAD — ${name}`,
    color:new THREE.Color().setRGB(r,g,b,THREE.LinearSRGBColorSpace),
    metalness:source.metallicFactor ?? 1,
    roughness:source.roughnessFactor ?? 1,
    opacity:alpha,transparent:alpha<1,
  });
}
