#!/usr/bin/env python
#
# This app should:
# - check whether there's a generated SSL certificate and key
# - generate the key and certificate
# - distribute the cert + key among all nodes that talk to wss://

import subprocess
import os


class SSLManager():
    def __init__(self):
        self.ssl_hosts = ['42-a', '42-b']
        self.home_dir = "/home/lg"
        self.catkin_home = "%s/catkin" % self.home_dir
        self.ssl_directory = "%s/ssl" % self.home_dir
        self.key_path = "%s/etc/slinky.key" % self.home_dir
        self.crt_path = "%s/etc/slinky.crt" % self.home_dir
        self.csr_path = "%s/etc/slinky.csr" % self.home_dir
        self.ssl_crt_gen_cmd = "openssl req -nodes -new -x509 \
                                -keyout %s/etc/slinky.key \
                                -out %s/etc/slinky.crt \
                                -days 3650 \
                                -config %s/etc/openssl.cfg" \
                                % (self.home_dir, self.home_dir, self.home_dir)
        self.ssl_local_crt_import_cmd = "rm -fr %s/.pki ; \
                                         mkdir -p %s/.pki/nssdb ; \
                                         chmod 700 %s/.pki/nssdb ; \
                                         certutil -d sql:%s/.pki/nssdb -N \
                                         --empty-password ;\
                                         certutil -d sql:%s/.pki/nssdb \
                                         -A -t P,P,P -n acme \
                                         -i %s/etc/slinky.crt \
                                         --empty-password" \
                                         % (self.home_dir,
                                            self.home_dir,
                                            self.home_dir,
                                            self.home_dir,
                                            self.home_dir,
                                            self.home_dir)
        self.nssdb_crt_check_cmd = "/usr/bin/certutil -d sql:%s/.pki/nssdb -L | \
        grep 'acme'" % (self.home_dir)

    def manage_ssl(self):
        ssl_distributed = False
        if not self._certificate_imported_to_nssdb():
            if self._generate_ssl_crt():
                ssl_distributed = self._distribute_ssl()
            if ssl_distributed:
                return self._import_ssl_remotely()
            else:
                return False

    def run(self):
        self.manage_ssl()

    def _distribute_ssl(self):
        for host in self.ssl_hosts:
            cmd = 'scp etc/slinky* %s:etc/' % host
            os.system(cmd)
        return True

    def _check_if_crt_exists(self):
        return os.path.isfile(self.crt_path)

    def _check_if_key_exists(self):
        return os.path.isfile(self.key_path)

    def _certificate_imported_to_nssdb(self):
        crt_imported_to_nssdb_on_all_hosts = False
        for host in self.ssl_hosts:
            cmd = 'ssh %s %s' % (host, self.nssdb_crt_check_cmd)
            cmd = cmd.split(' ')
            cmd = [item for item in cmd if item != '']
            print "Checking if SSL cert is imported into nssdb"
            p = subprocess.Popen(cmd,
                                 stdout=subprocess.PIPE,
                                 stderr=subprocess.PIPE)
            out, err = p.communicate()
            if 'P,P,P' in out:
                crt_imported_to_nssdb_on_all_hosts = True
            else:
                return False
        return crt_imported_to_nssdb_on_all_hosts

    def _generate_ssl_crt(self):
        if self._check_if_crt_exists():
            return False
        else:
            print "Generating SSL certificate"
            cmd = self.ssl_crt_gen_cmd.split(' ')
            cmd = [item for item in cmd if item != '']
            p = subprocess.Popen(cmd,
                                 stdout=subprocess.PIPE,
                                 stderr=subprocess.PIPE)
            out, err = p.communicate()
            print out
            print err
            return os.path.isfile(self.crt_path)

    def _import_ssl_remotely(self):
        print "Importing crt file remotely"
        for host in self.ssl_hosts:
            cmd = 'ssh %s %s' % (host, self.ssl_local_crt_import_cmd)
            cmd = cmd.split(' ')
            cmd = [item for item in cmd if item != '']
            print cmd
            p = subprocess.Popen(cmd,
                                 stdout=subprocess.PIPE,
                                 stderr=subprocess.PIPE)
            out, err = p.communicate()
            print out
            print err
        return True

if __name__ == '__main__':
    sm = SSLManager()
    sm.run()
