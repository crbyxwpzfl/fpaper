// DOM elements
const fileInput = document.getElementById('fileInput');
const uploadArea = document.getElementById('uploadArea');
const displaySection = document.getElementById('displaySection');
const uploadedImage = document.getElementById('uploadedImage');
const fileName = document.getElementById('fileName');
const fileSize = document.getElementById('fileSize');

// File upload event listeners
fileInput.addEventListener('change', handleFileSelect);
uploadArea.addEventListener('click', () => fileInput.click());

// Drag and drop functionality
uploadArea.addEventListener('dragover', handleDragOver);
uploadArea.addEventListener('drop', handleDrop);
uploadArea.addEventListener('dragenter', handleDragEnter);
uploadArea.addEventListener('dragleave', handleDragLeave);

// Prevent default drag behaviors
['dragenter', 'dragover', 'dragleave', 'drop'].forEach(eventName => {
    uploadArea.addEventListener(eventName, preventDefaults, false);
    document.body.addEventListener(eventName, preventDefaults, false);
});

function preventDefaults(e) {
    e.preventDefault();
    e.stopPropagation();
}

function handleDragOver(e) {
    uploadArea.classList.add('dragover');
}

function handleDragEnter(e) {
    uploadArea.classList.add('dragover');
}

function handleDragLeave(e) {
    uploadArea.classList.remove('dragover');
}

function handleDrop(e) {
    uploadArea.classList.remove('dragover');
    const files = e.dataTransfer.files;
    handleFiles(files);
}

function handleFileSelect(e) {
    const files = e.target.files;
    handleFiles(files);
}

function handleFiles(files) {
    console.log('handleFiles called with', files.length, 'files');
    if (files.length === 0) return;
    
    const file = files[0];
    console.log('Processing file:', file.name, 'Type:', file.type, 'Size:', file.size);
    
    // Validate file type
    if (!file.type.startsWith('image/')) {
        console.error('Invalid file type:', file.type);
        alert('Please select a valid image file.');
        return;
    }
    
    // Validate file size (10MB max)
    const maxSize = 10 * 1024 * 1024; // 10MB
    if (file.size > maxSize) {
        console.error('File too large:', file.size);
        alert('File size must be less than 10MB.');
        return;
    }
    
    console.log('File validation passed, calling displayPhoto');
    displayPhoto(file);
}

function displayPhoto(file) {
    console.log('displayPhoto called with file:', file.name);
    const reader = new FileReader();
    
    reader.onload = function(e) {
        console.log('File reader onload triggered');
        
        // Set image source first
        uploadedImage.src = e.target.result;
        
        // Set file information
        fileName.textContent = file.name;
        fileSize.textContent = formatFileSize(file.size);
        
        // Hide upload section and show display section
        document.querySelector('.upload-section').style.display = 'none';
        displaySection.style.display = 'block';
        
        console.log('Image displayed successfully');
    };
    
    reader.onerror = function(error) {
        console.error('FileReader error:', error);
        alert('Error reading file. Please try again.');
    };
    
    reader.readAsDataURL(file);
}

function formatFileSize(bytes) {
    if (bytes === 0) return '0 Bytes';
    
    const k = 1024;
    const sizes = ['Bytes', 'KB', 'MB', 'GB'];
    const i = Math.floor(Math.log(bytes) / Math.log(k));
    
    return parseFloat((bytes / Math.pow(k, i)).toFixed(2)) + ' ' + sizes[i];
}

function clearPhoto() {
    // Reset file input
    fileInput.value = '';
    
    // Clear image
    uploadedImage.src = '';
    
    // Show upload section and hide display section
    document.querySelector('.upload-section').style.display = 'block';
    displaySection.style.display = 'none';
    
    // Remove dragover class if present
    uploadArea.classList.remove('dragover');
}

// Add some debugging on page load
document.addEventListener('DOMContentLoaded', function() {
    console.log('DOM loaded');
    console.log('Upload area:', document.getElementById('uploadArea'));
    console.log('Display section:', document.getElementById('displaySection'));
    console.log('Uploaded image:', document.getElementById('uploadedImage'));
    
    // Add entrance animation to container
    const container = document.querySelector('.container');
    container.style.opacity = '0';
    container.style.transform = 'translateY(30px)';
    
    setTimeout(() => {
        container.style.transition = 'all 0.6s ease';
        container.style.opacity = '1';
        container.style.transform = 'translateY(0)';
    }, 100);
});

// Add keyboard support
document.addEventListener('keydown', function(e) {
    // Press 'U' to upload
    if (e.key.toLowerCase() === 'u' && !e.ctrlKey && !e.altKey) {
        if (displaySection.style.display === 'none') {
            fileInput.click();
        }
    }
    
    // Press 'Escape' to clear photo
    if (e.key === 'Escape') {
        if (displaySection.style.display !== 'none') {
            clearPhoto();
        }
    }
});

// Add paste support for images
document.addEventListener('paste', function(e) {
    const items = e.clipboardData.items;
    
    for (let i = 0; i < items.length; i++) {
        const item = items[i];
        
        if (item.type.startsWith('image/')) {
            const file = item.getAsFile();
            displayPhoto(file);
            break;
        }
    }
});
