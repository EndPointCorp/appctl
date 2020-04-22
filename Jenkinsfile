pipeline {
  environment {
    APTLY_SERVER = credentials('aptly-server-url')
  }
  agent {
    dockerfile {
      args "-u 0 -v /var/lib/jenkins/.ssh:/root/ssh"
      additionalBuildArgs '--build-arg BUILD_DEBS=true'
    }
  }
  stages {
    stage('Test') {
      steps {
        sh "./scripts/test.sh"
      }
    }
    stage('Build') {
      when {
        branch 'master'
      }
      steps {
        sh "./scripts/build.sh"
      }
    }
    stage('Deploy') {
      when {
        branch 'mater'
      }
      steps {
        sh "./scripts/deploy.sh"
      }
    }
  }
}
