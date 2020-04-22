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
        sh "./scripts/run_tests.sh"
      }
    }
    stage('Build') {
      when {
        branch 'master'
      }
      steps {
        sh "./pack-debs"
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
