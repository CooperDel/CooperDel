import { MeshoptDecoder } from 'three/examples/jsm/libs/meshopt_decoder.module.js';
import * as THREE from 'three';
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js';
import { GLTFLoader } from 'three/examples/jsm/loaders/GLTFLoader.js';
import { STLLoader } from 'three/examples/jsm/loaders/STLLoader.js';
import { OBJLoader } from 'three/examples/jsm/loaders/OBJLoader.js';
import { ColladaLoader } from 'three/examples/jsm/loaders/ColladaLoader.js';
import URDFLoader from 'urdf-loader';
import { resolveSelectedFile, jointRange, jointValueAt } from './model-utils.js';
import { createNovaMotion } from './nova-motion.js';
import { createRoboNavMotion } from './robonav-motion.js';
import { createUUVMotion } from './uuv-motion.js';
import { createV3FleetMotion } from './v3-fleet-motion.js';
import { RoomEnvironment } from 'three/examples/jsm/environments/RoomEnvironment.js';
import './image-lightbox.js';
import { createV3PayloadMotion } from './v3-payload-motion.js';


const extension = name => name.split(/[?#]/)[0].split('.').pop().toLowerCase();
function release(object) {
  const geometries = new Set(), materials = new Set(), textures = new Set();
  object?.traverse(child => {
    if (child.geometry) geometries.add(child.geometry);
    for (const material of (Array.isArray(child.material) ? child.material : [child.material])) {
      if (!material) continue;
      materials.add(material);
      for (const value of Object.values(material)) if (value?.isTexture) textures.add(value);
    }
  });
  textures.forEach(t => { t.dispose(); t.source?.data?.close?.(); });
  materials.forEach(m => m.dispose()); geometries.forEach(g => g.dispose());
}
function validateURDF(text) {
  const xml = new DOMParser().parseFromString(text, 'application/xml');
  if (xml.querySelector('parsererror') || xml.documentElement.tagName !== 'robot') throw new Error('This is not a valid URDF robot XML file. Export expanded URDF, not Xacro.');
  if (text.includes('xacro:') || text.includes('${')) throw new Error('Expand Xacro to URDF before loading.');
  const links = [...xml.documentElement.children].filter(n => n.tagName === 'link');
  const joints = [...xml.documentElement.children].filter(n => n.tagName === 'joint');
  const names = links.map(n => n.getAttribute('name'));
  if (!names.length || names.some(n => !n) || new Set(names).size !== names.length) throw new Error('URDF links must have unique names.');
  const parents = new Map(), jointNames = new Set();
  for (const joint of joints) {
    const name = joint.getAttribute('name');
    const parent = joint.querySelector('parent')?.getAttribute('link');
    const child = joint.querySelector('child')?.getAttribute('link');
    if (!name || jointNames.has(name) || !names.includes(parent) || !names.includes(child) || parents.has(child)) throw new Error('Invalid URDF joint names or parent/child links.');
    jointNames.add(name); parents.set(child, parent);
  }
  for(const joint of joints){const source=joint.querySelector('mimic')?.getAttribute('joint');if(source&&!jointNames.has(source))throw new Error(`Mimic joint references a missing joint: ${source}`);}
  for (const name of names) {
    const visited = new Set(); let node = name;
    while (node) { if (visited.has(node)) throw new Error('The URDF link tree contains a cycle.'); visited.add(node); node = parents.get(node); }
  }
  if (names.filter(name => !parents.has(name)).length !== 1) throw new Error('The URDF must have one connected root link.');
  return xml;
}

async function initialize(root) {
  const find = selector => root.querySelector(selector);
  const robotMode = root.dataset.kind === 'urdf';
  const config = JSON.parse(root.dataset.config);
  const stage = find('[data-stage]'), empty = find('[data-empty]'), status = find('[data-status]');
  const report = (message, error = false) => { status.textContent = message; status.dataset.error = String(error); if (root.dataset.presentationOnly === 'true') status.hidden = !error; };
  let renderer;
  const fleetRendering=['v3-lunar-fleet','v3-yam-payload'].includes(config.motion);
  const lunarFleet = config.motion === 'v3-lunar-fleet';
  try { renderer = new THREE.WebGLRenderer({ antialias:!fleetRendering, alpha:false, powerPreference:'high-performance' }); }
  catch { report('3D graphics are unavailable. Enable hardware acceleration or use a browser with WebGL 2 support.', true); return; }
  renderer.setPixelRatio(fleetRendering ? 1.25 : Math.min(devicePixelRatio,2));
  renderer.setClearColor(0x11191e);
  renderer.outputColorSpace = THREE.SRGBColorSpace;
  renderer.toneMapping = THREE.NoToneMapping;
  if(config.motion === 'v3-lunar-fleet') { renderer.shadowMap.enabled=true;renderer.shadowMap.type=THREE.PCFSoftShadowMap; }
  renderer.domElement.setAttribute('aria-label', robotMode ? `Interactive ${root.dataset.modelLabel || 'robot'} model` : `Interactive ${root.dataset.modelLabel || 'CAD'} model`);
  stage.append(renderer.domElement);
  const scene = new THREE.Scene();
  let environmentMap=null;
  if(fleetRendering||config.cadLighting){
    const generator=new THREE.PMREMGenerator(renderer),room=new RoomEnvironment();
    environmentMap=generator.fromScene(room,.04);scene.environment=environmentMap.texture;scene.environmentIntensity=.45;room.dispose();generator.dispose();
    window.addEventListener('pagehide',()=>environmentMap.dispose(),{once:true});
  }
  const softerLighting=lunarFleet||config.cadLighting;
  scene.add(new THREE.HemisphereLight(config.neutralLighting ? 0xffffff : 0xd7efff, config.neutralLighting ? 0x555555 : 0x45535b, softerLighting ? .65 : 3));
  const key = new THREE.DirectionalLight(0xffffff, softerLighting ? .8 : 3); key.position.set(3,6,5); scene.add(key);
  const fill = new THREE.DirectionalLight(config.neutralLighting ? 0xffffff : 0x9be7ec, softerLighting ? .2 : 2); fill.position.set(-4,2,-3); scene.add(fill);
  const camera = new THREE.PerspectiveCamera(40, 1, .001, 10000);
  camera.position.set(3,2,3);
  const controls = new OrbitControls(camera, renderer.domElement);
  controls.enableDamping = true;
  if(Number.isFinite(config.autoRotateSpeed))controls.autoRotateSpeed=config.autoRotateSpeed;
  if(config.scrollNavigation){controls.enableZoom=false;renderer.domElement.style.touchAction='pan-y';}
  const scrollSection = config.scrollSpin === false ? null : root.closest(config.scrollSpinInOut ? '.design-intro-with-model' : '.scroll-design');
  const reducedMotion = window.matchMedia('(prefers-reduced-motion: reduce)');
  let scrollAngle = 0;
  if (scrollSection) {
    controls.enableZoom = false;
    renderer.domElement.style.touchAction = 'pan-y';
  }
  let model = null, robot = null, jointRows = [], playing = false, time = 0, previous = 0, generation = 0;
  let activeURLs = [], selectedFiles = [], selectedURDFs = [];
  let motion = null;
  let radius = 1, center = new THREE.Vector3(), visible = true;
  const cameraButtons = ['reset','left','right','in','out'];
  const buttonsEnabled = enabled => cameraButtons.forEach(name => { const button=find(`[data-${name}]`); if(button) button.disabled = !enabled; });
  const stop = () => { playing = false; if (robotMode) { find('[data-play]').textContent = config.motion ? 'Play motion' : 'Play joints'; find('[data-play]').setAttribute('aria-pressed','false'); } };
  const fit = () => {
    if (!model) return;
    model.updateMatrixWorld(true);
    const box = new THREE.Box3().setFromObject(model);
    if (box.isEmpty()) throw new Error('The model has no visible geometry. Check the URDF visual elements and meshes.');
    box.getCenter(center); radius = box.getSize(new THREE.Vector3()).length() / 2;
    if (!Number.isFinite(radius) || radius <= 0) throw new Error('The model has invalid dimensions.');
    const limitingFov = Math.min(camera.fov * Math.PI / 180, 2 * Math.atan(Math.tan(camera.fov * Math.PI / 360) * camera.aspect));
    const distance = radius / Math.sin(limitingFov / 2) * 1.15 * (motion?.viewScale || 1) * (Number.isFinite(config.viewScale)&&config.viewScale>0?config.viewScale:1) * (scrollSection ? .88 : 1);
    camera.near = radius / 1000; camera.far = radius * 1000; camera.updateProjectionMatrix();
    const viewDirection = config.view === 'isometric' ? new THREE.Vector3(1,1,1) : new THREE.Vector3(1,.7,1);
    if (Array.isArray(config.viewDirection) && config.viewDirection.length === 3 && config.viewDirection.every(Number.isFinite) && config.viewDirection.some(value => value !== 0)) {
      viewDirection.fromArray(config.viewDirection);
    }
    camera.position.copy(center).add(viewDirection.normalize().multiplyScalar(distance));
    const initialCamera=config.initialCamera;
    if(initialCamera&&[initialCamera.position,initialCamera.target].every(v=>Array.isArray(v)&&v.length===3&&v.every(Number.isFinite))){
      center.fromArray(initialCamera.target);
      const offset=new THREE.Vector3().fromArray(initialCamera.position).sub(center);
      // Keep the screenshot's framing on desktop and allow more room on narrow screens.
      offset.multiplyScalar(Math.max(1,(initialCamera.referenceAspect||camera.aspect)/camera.aspect));
      camera.position.copy(center).add(offset);
      motion?.cameraTarget?.copy(center);
    }
    controls.target.copy(center); controls.minDistance = radius * .05; controls.maxDistance = radius * 100;
    controls.update();
  };
  const resize = () => { const w = stage.clientWidth, h = stage.clientHeight; renderer.setSize(w,h,false); camera.aspect=w/h; camera.updateProjectionMatrix(); };
  const resizeObserver = new ResizeObserver(resize); resizeObserver.observe(stage); resize();
  const observer = new IntersectionObserver(entries => { visible = entries[0].isIntersecting; }); observer.observe(stage);
  const updateRows = () => jointRows.forEach(({joint,input,output}) => {
    const value = joint.jointValue[0] || 0;
    input.value = value;
    output.textContent = joint.jointType === 'prismatic' ? `${value.toFixed(3)} m` : `${THREE.MathUtils.radToDeg(value).toFixed(1)}°`;
  });
  const setupJoints = () => {
    const panel = find('[data-joints]'); panel?.replaceChildren(); jointRows = [];
    if (motion) {
      if(panel) panel.textContent = motion.description || 'Staggered rocks drive the suspension pivots and differential bar together. The rover stops and uses sideways crab steering to bypass the large block. Both rigid connecting links remain attached at their ends. Kinematic demonstration with idealized connector bearings; not a contact-physics simulation.';
      find('[data-play]').disabled=false; find('[data-pose]').disabled=false;
      return;
    }
    const joints = Object.values(robot.joints).filter(j => ['continuous','revolute','prismatic'].includes(j.jointType) && !j.mimicJoint);
    for (const [index,joint] of joints.entries()) {
      const [min,max] = jointRange(joint);
      if (!Number.isFinite(min) || !Number.isFinite(max) || max < min) throw new Error(`Invalid limits for joint ${joint.name}.`);
      const row = document.createElement('div'); row.className = 'joint-row';
      const heading = document.createElement('div'); heading.className = 'joint-title';
      const label = document.createElement('label'); label.htmlFor=`${root.id}-joint-${index}`; label.textContent=joint.name;
      const output = document.createElement('output'); output.htmlFor=label.htmlFor;
      const input = document.createElement('input'); input.type='range'; input.id=label.htmlFor; input.min=min; input.max=max; input.step=(max-min)/1000 || .001; input.disabled=max===min;
      input.addEventListener('input',()=>{stop(); joint.setJointValue(Number(input.value)); updateRows();});
      heading.append(label,output); row.append(heading,input); panel.append(row); jointRows.push({joint,input,output});
      joint.setJointValue(0);
    }
    if (!joints.length) { const p=document.createElement('p'); p.textContent='This robot has no movable revolute, continuous, or prismatic joints.'; panel.append(p); }
    find('[data-play]').disabled=!jointRows.some(r=>!r.input.disabled);
    find('[data-pose]').disabled=!jointRows.length;
    updateRows();
  };
  async function load(source, files = []) {
    const ticket = ++generation;
    motion?.dispose(); motion=null;
    stop(); buttonsEnabled(false); report('Loading model…'); empty.hidden=false; empty.textContent='Loading model…';
    if (robotMode) { find('[data-play]').disabled=true; find('[data-pose]').disabled=true; find('[data-joints]')?.replaceChildren(); }
    if (model) { scene.remove(model); release(model); model=null; }
    robot=null; jointRows=[];
    activeURLs.forEach(URL.revokeObjectURL); activeURLs=[];
    const urls=[], urlByFile=new Map(); let candidate=null;
    const filePath = source instanceof File ? (source.webkitRelativePath || source.name) : source;
    const base = filePath.slice(0,filePath.lastIndexOf('/')+1);
    const manager = new THREE.LoadingManager();
    const failures=[];
    manager.onError = url => failures.push(`Unable to read ${url.startsWith('blob:') ? 'a selected model asset' : url}`);
    const blob = file => { if (!urlByFile.has(file)) { const url=URL.createObjectURL(file); urlByFile.set(file,url); urls.push(url); } return urlByFile.get(file); };
    const resolve = request => {
      if (/^(blob:|data:)/.test(request)) return request;
      if (files.length) return blob(resolveSelectedFile(request,files,base));
      const resolved = new URL(request,location.origin + base);
      if (resolved.origin !== location.origin || !['/CooperDel/assets/','/CooperDel/GLBs/','/CooperDel/URDFs/'].some(prefix => resolved.pathname.startsWith(prefix))) throw new Error('Model assets must be local files under /CooperDel/assets/, /CooperDel/GLBs/, or /CooperDel/URDFs/. External URLs are not loaded.');
      return resolved.href;
    };
    manager.setURLModifier(resolve);
    const fetchText = async () => {
      if (source instanceof File) return source.text();
      const response = await fetch(resolve(source));
      if (!response.ok) throw new Error(`Model not found: ${source}`);
      return response.text();
    };
    const gltf = async (url) => (await new GLTFLoader(manager).setMeshoptDecoder(MeshoptDecoder).loadAsync(url)).scene;
    try {
      if (robotMode) {
        const xml = validateURDF(await fetchText());
        const loader = new URDFLoader(manager); loader.packages = name => files.length ? name : (config.packages?.[name] || `${base}${name}`);
        const jobs=[];
        loader.loadMeshCb = (url, loadingManager, material, done) => {
          if(url.includes('/CooperDel/optimized-meshes/')) url=url.slice(url.indexOf('/CooperDel/optimized-meshes/'));
          const job = (async()=>{
            try {
              let mesh;
              switch(extension(url)) {
                case 'stl': mesh=new THREE.Mesh(await new STLLoader(manager).loadAsync(url),material); break;
                case 'dae': mesh=(await new ColladaLoader(manager).loadAsync(url)).scene; break;
                case 'obj': mesh=await new OBJLoader(manager).loadAsync(url); mesh.traverse(c=>{if(c.isMesh)c.material=material;}); break;
                case 'glb': case 'gltf': mesh=await gltf(url); if(config.materialFromURDF || url.includes('/CooperDel/optimized-meshes/'))mesh.traverse(c=>{if(c.isMesh)c.material=material;}); break;
                default: throw new Error(`Unsupported URDF mesh: ${url}. Use STL, DAE, OBJ, or GLB.`);
              }
              done(mesh);
            } catch(error) { failures.push(error.message); done(null,error); }
          })(); jobs.push(job);
        };
        // One sentinel keeps texture completion in the same loading batch.
        const complete = new Promise(resolveDone => { manager.onLoad=resolveDone; });
        manager.itemStart('robot-load');
        candidate=loader.parse(xml,files.length ? '' : base);
        const parallel=xml.querySelector('parallel_manipulator');
        if(parallel)candidate.userData.parallelManipulator={
          carrier:parallel.getAttribute('carrier'),endEffector:parallel.getAttribute('end_effector'),
          anchors:[...parallel.querySelectorAll('anchor')].map(n=>n.getAttribute('link')),
          closures:[...parallel.querySelectorAll('closure')].map(n=>[n.getAttribute('link_a'),n.getAttribute('link_b')])
        };
        await Promise.all(jobs); manager.itemEnd('robot-load'); await complete;
        candidate.rotation.x=config.upAxis === 'negative-y' ? Math.PI : -Math.PI/2;
      } else {
        if (extension(filePath) !== 'glb') throw new Error('Choose a .glb export with embedded geometry and textures.');
        const assembly=await gltf(source instanceof File ? blob(source) : source);
        // A parent transform preserves the CAD assembly's original node transforms.
        candidate=new THREE.Group();
        candidate.add(assembly);
        if (Array.isArray(config.orientation) && config.orientation.length === 4 && config.orientation.every(Number.isFinite)) {
          candidate.quaternion.fromArray(config.orientation).normalize();
        }
        if (config.upAxis === 'z') candidate.rotation.x=config.upAxis === 'negative-y' ? Math.PI : -Math.PI/2;
        if (Number.isFinite(config.initialYaw)) candidate.rotateOnWorldAxis(new THREE.Vector3(0,1,0),config.initialYaw);
        if (Number.isFinite(config.initialPitch)) candidate.rotateOnWorldAxis(new THREE.Vector3(1,0,0),config.initialPitch);
      }
      if (ticket !== generation) { release(candidate); urls.forEach(URL.revokeObjectURL); return; }
      if (failures.length) throw new Error(failures.slice(0,4).join(' '));
      model=candidate; scene.add(model);
      if (robotMode) {
        robot=model;
        if(config.motion === 'nova-swerve') motion=createNovaMotion(robot,scene);
        if(config.motion === 'v3-lunar-fleet') motion=createV3FleetMotion(robot,scene);
        if(config.motion === 'v3-yam-payload') motion=createV3PayloadMotion(robot,scene);
        if(config.motion === 'uuv-swim') motion=createUUVMotion(robot,scene);
        if(config.motion === 'robonav-mars') motion=createRoboNavMotion(robot,scene);
        setupJoints();
        if (motion && !matchMedia('(prefers-reduced-motion: reduce)').matches) {
          playing=true; find('[data-play]').textContent='Pause motion'; find('[data-play]').setAttribute('aria-pressed','true');
        }
      }
      fit();
      activeURLs=urls; empty.hidden=true; buttonsEnabled(true);
      report(`${filePath.split('/').pop()} loaded.${robotMode ? ` ${jointRows.length} controllable joints.` : ''} ${files.length ? 'Local preview only; reload clears this selection.' : 'Portfolio model loaded from local assets.'}`);
    } catch(error) {
      if (ticket !== generation) { release(candidate); urls.forEach(URL.revokeObjectURL); return; }
      motion?.dispose(); motion=null;
      if (candidate) { scene.remove(candidate); release(candidate); }
      model=null; robot=null; jointRows=[]; urls.forEach(URL.revokeObjectURL);
      empty.hidden=false; empty.textContent='Model could not be loaded';
      if (robotMode) { find('[data-joints]')?.replaceChildren(); find('[data-play]').disabled=true; find('[data-pose]').disabled=true; }
      report(error.message || 'Unable to load the model. Use an uncompressed GLB with embedded textures.',true);
    }
  }
  function selected(list) {
    selectedFiles=Array.from(list); if (!selectedFiles.length) return;
    if (!robotMode) { load(selectedFiles[0],selectedFiles); return; }
    selectedURDFs=selectedFiles.filter(file=>extension(file.name)==='urdf');
    if (!selectedURDFs.length) { report('No .urdf file found. Choose an expanded URDF together with its meshes.',true); return; }
    const select=find('[data-robot-select]'); select.replaceChildren();
    selectedURDFs.forEach((file,i)=>{const option=document.createElement('option');option.value=i;option.textContent=file.webkitRelativePath||file.name;select.append(option);});
    find('[data-select-wrap]').hidden=selectedURDFs.length<2;
    load(selectedURDFs[0],selectedFiles);
  }
  find('[data-files]')?.addEventListener('change',event=>selected(event.target.files));
  if (robotMode) {
    find('[data-folder]')?.addEventListener('change',event=>selected(event.target.files));
    find('[data-robot-select]')?.addEventListener('change',event=>load(selectedURDFs[Number(event.target.value)],selectedFiles));
    find('[data-play]').addEventListener('click',()=>{playing=!playing;find('[data-play]').textContent=playing?'Pause motion':'Play motion';find('[data-play]').setAttribute('aria-pressed',String(playing));});
    find('[data-pose]').addEventListener('click',()=>{stop();time=0;motion?.reset();jointRows.forEach(r=>r.joint.setJointValue(0));updateRows();});
  }
  find('[data-reset]').addEventListener('click',fit);
  for (const [name,angle] of [['left',-.25],['right',.25]]) find(`[data-${name}]`)?.addEventListener('click',()=>{camera.position.sub(controls.target).applyAxisAngle(new THREE.Vector3(0,1,0),angle).add(controls.target);controls.update();});
  for (const [name,factor] of [['in',.8],['out',1.25]]) find(`[data-${name}]`)?.addEventListener('click',()=>{camera.position.sub(controls.target).multiplyScalar(factor).add(controls.target);controls.update();});
  let frameAverage=1/60,qualityFrames=0;
  renderer.setAnimationLoop(now=>{
    const frameDuration=previous ? (now-previous)/1000 : 1/60;
    const delta=Math.min((now-previous)/1000,.05); previous=now;
    if (!visible || document.hidden) return;
    if(fleetRendering && frameDuration>0 && frameDuration<.25){
      frameAverage=frameAverage*.96+frameDuration*.04;
      if(++qualityFrames>=45){
        qualityFrames=0;const current=renderer.getPixelRatio();
        const next=frameAverage>.022 ? Math.max(.85,current-.1) : frameAverage<.0165 ? Math.min(1.5,current+.025) : current;
        if(Math.abs(next-current)>.01){renderer.setPixelRatio(next);resize();}
      }
    }
    if (scrollSection) {
      const rect = scrollSection.getBoundingClientRect();
      const progress = reducedMotion.matches ? 1 : THREE.MathUtils.clamp((innerHeight * .65 - rect.top) / (innerHeight * .85), 0, 1);
      const eased = progress * progress * (3 - 2 * progress);
      scrollSection.style.setProperty('--arrival', eased);
      const exitProgress = config.scrollSpinInOut && !reducedMotion.matches ? THREE.MathUtils.clamp((innerHeight * .9 - rect.bottom) / (innerHeight * .65), 0, 1) : 0;
      const exitEased = exitProgress * exitProgress * (3 - 2 * exitProgress);
      if(config.scrollSpinInOut)scrollSection.style.setProperty('--model-presence', Math.min(eased,1-exitEased));
      const angle = (eased + exitEased) * Math.PI * 2;
      if (model && angle !== scrollAngle) {
        camera.position.sub(controls.target).applyAxisAngle(new THREE.Vector3(0,1,0), angle - scrollAngle).add(controls.target);
        scrollAngle = angle;
      }
      controls.enabled = progress >= 1;
      // Only the subsection occupying the center of the viewport keeps turning.
      // OrbitControls pauses automatic rotation while the user drags the model.
      controls.autoRotate=Number.isFinite(config.autoRotateSpeed)&&!reducedMotion.matches&&progress>=1&&rect.top<=innerHeight*.5&&rect.bottom>=innerHeight*.5;
    }
    if(!scrollSection&&Number.isFinite(config.autoRotateSpeed))controls.autoRotate=!reducedMotion.matches;
    if (playing && robot) { const step=delta*Number(find('[data-speed]').value);time+=step; if(motion)motion.update(time,step);else jointRows.forEach(r=>r.joint.setJointValue(jointValueAt(r.joint,time)));updateRows(); }
    const phaseLabel=find('[data-motion-phase]');
    if(motion && phaseLabel && phaseLabel.textContent!==motion.phase)phaseLabel.textContent=motion.phase;
    if(motion?.cameraTarget) {
      const offset=motion.cameraTarget.clone().sub(center);
      camera.position.add(offset);controls.target.add(offset);center.copy(motion.cameraTarget);
    }
    controls.update(delta); renderer.render(scene,camera);
  });
  window.addEventListener('pagehide',()=>{generation++;renderer.setAnimationLoop(null);resizeObserver.disconnect();observer.disconnect();controls.dispose();motion?.dispose();release(model);renderer.dispose();activeURLs.forEach(URL.revokeObjectURL);},{once:true});
  window.addEventListener('pageshow',event=>{if(event.persisted)location.reload();});
  if (config.src) await load(config.src);
}
const startViewer=root=>initialize(root).catch(error=>{
  const status=root.querySelector('[data-status]');status.textContent=`Viewer could not start: ${error.message}`;status.dataset.error='true';status.hidden=false;
});
for (const root of document.querySelectorAll('.model-viewer')) {
  if(root.closest('.scroll-payload')){
    const observer=new IntersectionObserver(entries=>{if(entries.some(e=>e.isIntersecting)){observer.disconnect();startViewer(root);}},{rootMargin:'350px'});observer.observe(root);
  }else startViewer(root);
}

const revealSections = [...document.querySelectorAll('.scroll-design-intro')];
if (revealSections.length) {
  const preference = matchMedia('(prefers-reduced-motion: reduce)');
  let pending = false;
  const update = () => {
    pending = false;
    for (const section of revealSections) {
      const progress = preference.matches ? 1 : THREE.MathUtils.clamp((innerHeight * .8 - section.getBoundingClientRect().top) / (innerHeight * .65), 0, 1);
      section.style.setProperty('--reveal', progress * progress * (3 - 2 * progress));
    }
  };
  const schedule = () => { if (!pending) { pending = true; requestAnimationFrame(update); } };
  window.addEventListener('scroll', schedule, { passive:true });
  window.addEventListener('resize', schedule);
  preference.addEventListener('change', schedule);
  update();
}



