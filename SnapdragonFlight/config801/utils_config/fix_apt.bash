sudo apt-get -f install

sudo cp /usr/share/glib-2.0/schemas/gschema.dtd /usr/share/glib-2.0/schemas/gschema.dtd.bak

sudo cp /usr/bin/gdk-pixbuf-pixdata /usr/bin/gdk-pixbuf-pixdata.bak
sudo cp /usr/bin/gdk-pixbuf-query-loaders /usr/bin/gdk-pixbuf-query-loaders.bak

sudo apt-get install -y --force-yes     \
     -o Dpkg::Options::="--force-overwrite" \
     libgtk2.0-dev                      \
     libxcb1=1.10-2ubuntu1              \
     libxdamage1=1:1.1.4-1ubuntu1       \
     libxfixes3=1:5.0.1-1ubuntu1        \
     libxcursor1=1:1.1.14-1             \
     libxcb-shm0=1.10-2ubuntu1          \
     libxcb-render0=1.10-2ubuntu1       \
     libpixman-1-0=0.30.2-2ubuntu1      \
     libxrender1=1:0.9.8-1              \
     libfreetype6=2.5.2-1ubuntu2        \
     libfontconfig1=2.11.0-0ubuntu4     \
     fontconfig-config=2.11.0-0ubuntu4  \
     libxft2=2.3.1-2                    \
     libxcomposite1=1:0.4.4-1

#sudo dpkg --remove gdk-pixbuf libgdk-pixbuf2.0-commoninstall libgdk-pixbuf2.0-0:armhf
