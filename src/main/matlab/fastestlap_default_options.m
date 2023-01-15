function opts = fastestlap_default_options(type)

    if ( strcmpi(type,'create_track') )
        opts = '<options> <output_variables> <prefix>track/</prefix> <variables> <s/> </variables> </output_variables> </options>';
    else
        error('fastestlap_default_options(type) -> Type not recognized');
    end
end