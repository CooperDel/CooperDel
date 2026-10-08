const frame = document.querySelector('[data-pdf]');
const status = document.querySelector('.pdf-status');
let documentUrl;
try {
  const response = await fetch(frame.dataset.pdf);
  if (!response.ok) throw new Error(`PDF request failed: ${response.status}`);
  // Supply the PDF MIME type even when an older preview server returns octet-stream.
  const pdf = new Blob([await response.arrayBuffer()], { type: 'application/pdf' });
  documentUrl = URL.createObjectURL(pdf);
  frame.src = `${documentUrl}#view=FitH`;
  frame.hidden = false;
  status.hidden = true;
} catch (error) {
  status.textContent = 'The paper could not load. Please refresh the page to try again.';
  console.error(error);
}
window.addEventListener('pagehide', () => {
  if (documentUrl) URL.revokeObjectURL(documentUrl);
}, { once: true });
