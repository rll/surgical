function directory = RemoteHomeDirectory

if strcmp(getenv('OSTYPE'),'linux')
    directory = '/home/jinna/surgical/tracking/';               % I have pabbeel@surgical1:~/rll/code/trunk/surgical mounted to /home/jinna/surgical.
else
    directory = 'oops! mount filesystem on windows? ';
end
