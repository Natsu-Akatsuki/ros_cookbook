<!-- 移除命令行粘贴时无用的符号 -->
document.addEventListener('copy', function (event) {
    var selection = window.getSelection();
    var modifiedText = '';

    for (var i = 0; i < selection.rangeCount; i++) {
        var range = selection.getRangeAt(i);
        var commonAncestor = range.commonAncestorContainer;

        // 确保选中的内容来自于具有 class="lang-bash" 的 <code> 标签
        if (commonAncestor.nodeType !== Node.ELEMENT_NODE) {
            commonAncestor = commonAncestor.parentNode;
        }
        if (commonAncestor.tagName === 'CODE' && commonAncestor.classList.contains('lang-bash')) {
            // 删除粘贴内容里开头的 $ 或 # 或 () $ 或 () #
            modifiedText += range.toString().replace(/^[$#]?/gm, '');
        } else {
            // 如果不是来自于指定的标签，则保留原始文本
            modifiedText += range.toString();
        }
    }

    event.clipboardData.setData('text/plain', modifiedText);
    event.preventDefault(); // 阻止默认的复制行为
});
