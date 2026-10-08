// Pure helpers shared by the viewer and its local checks.
export function normalizePath(path) {
  const parts = decodeURIComponent(path).replace(/\\/g, '/').split('/');
  const result = [];
  for (const part of parts) {
    if (!part || part === '.') continue;
    if (part === '..') result.pop(); else result.push(part);
  }
  return result.join('/');
}
export function resolveSelectedFile(request, files, base = '') {
  const clean = request.replace(/^package:\/\//, '').replace(/^file:\/\//, '');
  const wanted = normalizePath(clean);
  const relative = normalizePath(`${base}/${clean}`);
  const entries = files.map(file => ({ file, path:normalizePath(file.webkitRelativePath || file.name) }));
  for (const candidate of [relative, wanted]) {
    const exact = entries.filter(entry => entry.path === candidate);
    if (exact.length === 1) return exact[0].file;
  }
  // Folder picks include an outer directory; match a complete suffix first.
  const suffix = entries.filter(entry => entry.path.endsWith('/' + wanted) || wanted.endsWith('/' + entry.path));
  if (suffix.length === 1) return suffix[0].file;
  const basename = wanted.split('/').pop();
  const matches = entries.filter(entry => entry.path.split('/').pop() === basename);
  if (matches.length === 1) return matches[0].file;
  throw new Error(matches.length ? `More than one file matches ${request}. Choose the whole robot folder with its original paths.` : `Missing file: ${request}. Select the URDF and all referenced meshes/textures together.`);
}
export function jointRange(joint) {
  if (joint.jointType === 'continuous') return [-Math.PI, Math.PI];
  return [Number(joint.limit.lower), Number(joint.limit.upper)];
}
export function jointValueAt(joint, seconds) {
  if (joint.jointType === 'continuous') return ((seconds + Math.PI) % (2 * Math.PI)) - Math.PI;
  const [lower, upper] = jointRange(joint);
  return (lower + upper) / 2 + Math.sin(seconds) * (upper - lower) / 2;
}
