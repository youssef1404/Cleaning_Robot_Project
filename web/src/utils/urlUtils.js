export const normalizeUrl = (url) => {
  if (!url) return '';
  
  // Remove trailing slashes
  let normalizedUrl = url.trim().replace(/\/+$/, '');
  
  // Ensure URL starts with http:// or https://
  if (!normalizedUrl.startsWith('http://') && !normalizedUrl.startsWith('https://')) {
    normalizedUrl = `https://${normalizedUrl}`;
  }
  
  return normalizedUrl;
};