document.addEventListener('copy', function (event) {
    var selection = window.getSelection().toString();
    var modifiedText = selection.replace(/^\$\s+/gm, ''); // 删除每行开头的 `$` 和空格
    modifiedText = modifiedText.replace(/^#\s+/gm, ''); // 删除每行开头的 `#` 和空格
    event.clipboardData.setData('text/plain', modifiedText);
    event.preventDefault(); // 阻止默认的复制行为
});
