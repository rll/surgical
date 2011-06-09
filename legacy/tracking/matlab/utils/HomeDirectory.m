function directory = HomeDirectory

if strcmp(getenv('OSTYPE'),'linux')
    directory = '/windows/D/Research/rll/code/trunk/surgical/tracking/';
else
    directory = 'D:\\Research\\rll\\code\\trunk\\surgical\\tracking\\';
end
