// Minimal: Videos nur abspielen, wenn sichtbar; Tastaturnavigation ←/→ auf Eintragsseiten.
document.querySelectorAll('figure.video video').forEach(v=>{
  v.controls=false; v.addEventListener('click',()=>v.paused?v.play():v.pause());
  if('IntersectionObserver' in window){
    new IntersectionObserver(es=>es.forEach(e=>e.isIntersecting?v.play().catch(()=>{}):v.pause()),{threshold:.4}).observe(v);
  }
});
document.addEventListener('keydown',e=>{
  if(e.target.matches('input,textarea')||e.metaKey||e.ctrlKey)return;
  const a=document.querySelector(e.key==='ArrowLeft'?'.pager .prev':e.key==='ArrowRight'?'.pager .next:not(.end)':null);
  if(a)location.href=a.href;
});
