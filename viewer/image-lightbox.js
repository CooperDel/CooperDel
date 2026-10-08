let dialog,invoker,previousOverflow;
document.addEventListener('click',event=>{
 const button=event.target.closest('[data-enlarge-src]');if(!button)return;
 if(!dialog){
  dialog=document.createElement('dialog');dialog.className='image-lightbox';dialog.setAttribute('aria-label','Enlarged project image');
  dialog.innerHTML='<button type="button" class="image-lightbox-close" aria-label="Close enlarged image">Close ×</button><img alt=""><p class="image-lightbox-caption"></p>';
  document.body.append(dialog);
  dialog.querySelector('button').addEventListener('click',()=>dialog.close());
  dialog.addEventListener('click',e=>{if(e.target!==dialog)return;const r=dialog.getBoundingClientRect();if(e.clientX<r.left||e.clientX>r.right||e.clientY<r.top||e.clientY>r.bottom)dialog.close();});
  dialog.addEventListener('close',()=>{document.documentElement.style.overflow=previousOverflow;invoker?.focus({preventScroll:true});});
 }
 invoker=button;const img=dialog.querySelector('img');img.src=button.dataset.enlargeSrc;img.alt=button.dataset.enlargeAlt;
 dialog.querySelector('p').textContent=img.alt;previousOverflow=document.documentElement.style.overflow;document.documentElement.style.overflow='hidden';dialog.showModal();
});
