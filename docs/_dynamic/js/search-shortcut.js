function focusSearchInput(event) {
    // Check if 'Ctrl' and '/' keys are pressed together
    if (event.ctrlKey && event.code === 'Slash') {
        event.preventDefault();
        // Get the search input element
        const searchInput = document.querySelector('.search input');
        if (searchInput) {
            searchInput.focus();
        }
    }
}

function initSearchShortcut() {
    // Listen for keydown events on the document
    document.addEventListener('keydown', focusSearchInput);
}

window.$docsify.plugins = [].concat(initSearchShortcut, window.$docsify.plugins);